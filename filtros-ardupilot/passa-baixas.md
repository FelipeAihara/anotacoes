# O filtro passa-baixas no ArduPilot: do capacitor ao Pixhawk 6C

## 1. Por que filtrar

Um quadricóptero é, do ponto de vista do giroscópio, uma máquina de vibrar. Quatro motores girando a algumas
centenas de hertz, hélices levemente desbalanceadas e um frame que flexiona produzem um espectro de ruído
que fica, quase todo, muito acima da banda em que o controlador de atitude realmente precisa atuar
(algumas dezenas de hertz, no máximo). Se esse ruído chega ao PID, o termo derivativo o amplifica, os
motores passam a responder a ele, esquentam e o voo fica "nervoso" — ou instável.

O passa-baixas existe para separar as duas coisas: deixar passar a dinâmica do veículo e atenuar o
resto. O preço é sempre o mesmo: **atraso de fase**. Toda a discussão de projeto de filtros no ArduPilot
gira em torno desse compromisso.

## 2. O caminho do sinal no Pixhawk 6C

O Pixhawk 6C usa um STM32H743 (Cortex-M7 a 480 MHz) e, nas revisões comuns, duas IMUs:
ICM-42688-P e BMI055 (revisões mais recentes trocam a segunda IMU; confira a sua com
`INS_GYR_ID`/`INS_GYR2_ID`). Simplificando, o caminho de um sinal de giroscópio no ArduCopter é:

```
sensor MEMS (filtro anti-aliasing interno, amostragem de kHz)
   │  SPI / FIFO
   ▼
AP_InertialSensor_Backend   ← taxa "crua" do backend (INS_GYRO_RATE: 1, 2, 4 ou 8 kHz)
   │  1. filtros notch harmônicos (INS_HNTCH_*, INS_HNTC2_*)
   │  2. notch estático (INS_NOTCH_*), se habilitado
   │  3. passa-baixas de 2ª ordem (INS_GYRO_FILTER)      ← LowPassFilter2p
   ▼
AP_InertialSensor (amostrado na taxa do loop, SCHED_LOOP_RATE = 400 Hz por padrão)
   ▼
AC_AttitudeControl → AC_PID do controlador de taxa
   │  passa-baixas de 1ª ordem no alvo (ATC_RAT_xxx_FLTT),
   │  no erro (FLTE) e no termo derivativo (FLTD)          ← LowPassFilter
   ▼
mixer → ESCs
```

Dois pontos importantes:

* **Os filtros da IMU rodam na taxa do backend, não na do loop.** Isso é o que o ArduPilot chama de
  *fast sampling*: o filtro "vê" o sinal a 1–8 kHz, de modo que o ruído de motor (100–400 Hz) está bem
  abaixo de Nyquist e pode ser atenuado *antes* de o sinal ser subamostrado para 400 Hz. Sem isso, o
  ruído de 300 Hz sofreria aliasing ao ser amostrado a 400 Hz e reapareceria como um falso sinal de
  100 Hz — e nenhum filtro digital posterior conseguiria distinguir isso de movimento real.
* **Existem dois tipos de passa-baixas no caminho:** um Butterworth de 2ª ordem (biquad) na IMU
  (`INS_GYRO_FILTER` e `INS_ACCEL_FILTER`, padrão 20 Hz) e filtros de 1ª ordem dentro dos PIDs.

## 3. A implementação: `LowPassFilter2p` (biquad)

O filtro de 2ª ordem está em `libraries/Filter/LowPassFilter2p.cpp`. Na essência (ArduPilot 4.x):

```cpp
void DigitalBiquadFilter<T>::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret)
{
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;
    if (!is_positive(ret.cutoff_freq)) {
        return;                        // fc = 0 → filtro desligado (passa direto)
    }
    float fr  = sample_freq / cutoff_freq;
    float ohm = tanf(M_PI / fr);                       // K = tan(π fc / fs)
    float c   = 1.0f + 2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    ret.b0 = ohm*ohm / c;
    ret.b1 = 2.0f * ret.b0;
    ret.b2 = ret.b0;
    ret.a1 = 2.0f*(ohm*ohm - 1.0f) / c;
    ret.a2 = (1.0f - 2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm) / c;
}

T DigitalBiquadFilter<T>::apply(const T &sample, const biquad_params &params)
{
    if (!initialised) {                // inicializa no regime permanente da 1ª amostra
        reset(sample, params);         // w1 = w2 = sample / (b0 + b1 + b2)
        initialised = true;
    }
    T w0  = sample - _delay_element_1*params.a1 - _delay_element_2*params.a2;
    T out = w0*params.b0 + _delay_element_1*params.b1 + _delay_element_2*params.b2;
    _delay_element_2 = _delay_element_1;
    _delay_element_1 = w0;
    return out;
}
```

Observações sobre o código:

* `2·cos(π/4) = √2` — é o coeficiente de amortecimento do Butterworth (ζ = 1/√2).
* A estrutura é a **Forma Direta II**: só dois estados (`w1`, `w2`) por eixo, em vez de quatro.
* O template `T` é `Vector3f` para giroscópio e acelerômetro, então os três eixos são filtrados com
  os mesmos coeficientes, de forma independente.
* A inicialização com `sample/(b0+b1+b2)` coloca os estados no valor de equilíbrio para uma entrada
  constante igual à primeira amostra, evitando um transitório no boot (ganho DC do filtro = 1, então a
  saída começa igual à entrada).

A equação a diferenças implementada é, portanto,

$$
w[n] = x[n] - a_1 w[n-1] - a_2 w[n-2], \qquad
y[n] = b_0 w[n] + b_1 w[n-1] + b_2 w[n-2],
$$

que equivale a

$$
H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}.
$$

## 4. De onde vem a equação: derivação

### 4.1 O protótipo analógico

O Butterworth de ordem *n* é definido como o filtro "maximamente plano": a resposta em magnitude
$|H(j\omega)|^2 = 1/(1 + (\omega/\omega_c)^{2n})$ tem as primeiras $2n-1$ derivadas nulas em
$\omega = 0$. Os polos de $H(s)H(-s)$ ficam igualmente espaçados sobre um círculo de raio $\omega_c$;
tomando os do semiplano esquerdo para $n = 2$ ($s = \omega_c e^{\pm j 3\pi/4}$), obtém-se

$$
H_a(s) = \frac{\omega_c^2}{s^2 + \sqrt{2}\,\omega_c\, s + \omega_c^2},
$$

isto é, um sistema de 2ª ordem com $\omega_n = \omega_c$ e $\zeta = 1/\sqrt{2} \approx 0{,}707$.
Em $\omega = \omega_c$ o ganho é exatamente $-3$ dB e a fase é $-90°$.

### 4.2 Transformação bilinear

Para levar $H_a(s)$ ao tempo discreto com período $T = 1/f_s$, usa-se a transformação bilinear
(Tustin), que vem de aproximar a integral pela regra do trapézio:

$$
s \;\leftarrow\; \frac{2}{T}\,\frac{1 - z^{-1}}{1 + z^{-1}}.
$$

Ela mapeia todo o eixo $j\omega$ analógico sobre o círculo unitário (logo, preserva estabilidade e não
causa aliasing do filtro), mas comprime as frequências de forma não linear:

$$
\omega_a = \frac{2}{T}\tan\!\left(\frac{\omega_d T}{2}\right).
$$

### 4.3 Pré-distorção (*prewarping*)

Para que o corte digital caia exatamente em $f_c$, projeta-se o protótipo analógico com a frequência
pré-distorcida $\omega_a = \frac{2}{T}\tan(\pi f_c / f_s)$. Definindo

$$
K \equiv \tan\!\left(\frac{\pi f_c}{f_s}\right) \quad(\texttt{ohm}\text{ no código}),
$$

a substituição fica particularmente limpa, porque os fatores $2/T$ se cancelam:

$$
\frac{s}{\omega_a} = \frac{1}{K}\,\frac{1 - z^{-1}}{1 + z^{-1}}.
$$

### 4.4 Álgebra

Escrevendo $H_a$ na forma normalizada $H_a = 1/\big((s/\omega_a)^2 + \sqrt2\,(s/\omega_a) + 1\big)$,
substituindo e multiplicando numerador e denominador por $K^2(1+z^{-1})^2$:

$$
H(z) = \frac{K^2 (1 + z^{-1})^2}
{(1 - z^{-1})^2 + \sqrt{2}\,K\,(1 - z^{-1})(1 + z^{-1}) + K^2 (1 + z^{-1})^2}.
$$

Expandindo o denominador por potências de $z^{-1}$:

| termo     | coeficiente                 |
|-----------|-----------------------------|
| $z^{0}$   | $1 + \sqrt2 K + K^2$        |
| $z^{-1}$  | $-2 + 2K^2$                 |
| $z^{-2}$  | $1 - \sqrt2 K + K^2$        |

Dividindo tudo por $c = 1 + \sqrt2 K + K^2$ para tornar o denominador mônico:

$$
b_0 = \frac{K^2}{c},\quad b_1 = 2b_0,\quad b_2 = b_0,\qquad
a_1 = \frac{2(K^2 - 1)}{c},\quad a_2 = \frac{1 - \sqrt2 K + K^2}{c}.
$$

Que são, linha a linha, os coeficientes de `compute_params`. Os dois zeros em $z = -1$ (vindos do
$(1+z^{-1})^2$) garantem ganho nulo em Nyquist; o ganho DC é $H(1) = 4K^2/(4K^2) = 1$.

Verificação numérica ($f_s = 2$ kHz, $f_c = 20$ Hz): $K = 0{,}0314$,
$b = [0{,}000945,\ 0{,}001889,\ 0{,}000945]$, $a = [1,\ -1{,}9112,\ 0{,}9150]$ — idêntico ao que
`scipy.signal.butter(2, 20, fs=2000)` produz.

> Note que $K \to \infty$ quando $f_c \to f_s/2$: o filtro só faz sentido com $f_c < f_s/2$, e na
> prática bem abaixo disso. Com o *fast sampling* isso raramente é problema, mas é um motivo para não
> configurar cortes altos em sensores lentos.

## 5. O filtro de 1ª ordem dos PIDs: `LowPassFilter`

Dentro de `AC_PID`, os filtros de alvo, erro e derivativo são de 1ª ordem
(`libraries/Filter/LowPassFilter.cpp`):

```cpp
float calc_lowpass_alpha_dt(float dt, float cutoff_freq)
{
    if (is_negative(dt) || is_negative(cutoff_freq)) { return 1.0; }
    if (is_zero(cutoff_freq)) { return 1.0; }   // desligado
    if (is_zero(dt))         { return 0.0; }
    float rc = 1.0f / (M_2PI * cutoff_freq);
    return dt / (dt + rc);
}

// apply:
_output += (sample - _output) * alpha;
```

### Derivação

É o circuito RC discretizado. A EDO é

$$
RC\,\dot y(t) + y(t) = x(t), \qquad RC = \frac{1}{2\pi f_c}.
$$

Aplicando **Euler implícito** (diferença para trás), $\dot y \approx (y_n - y_{n-1})/T$:

$$
RC\,\frac{y_n - y_{n-1}}{T} + y_n = x_n
\;\Longrightarrow\;
y_n = y_{n-1} + \underbrace{\frac{T}{T + RC}}_{\alpha}\,(x_n - y_{n-1}).
$$

Daí o `alpha = dt/(dt+rc)`. A solução exata (invariância ao degrau / ZOH) daria
$\alpha = 1 - e^{-T/RC}$; para $T \ll RC$, as duas coincidem em primeira ordem
($1 - e^{-\varepsilon} \approx \varepsilon/(1+\varepsilon)$ até $O(\varepsilon^2)$). A vantagem prática do
Euler implícito é ser incondicionalmente estável e barato, além de aceitar um `dt` variável a cada chamada
— útil porque o loop não tem período perfeitamente constante.

Em termos de $z$: $H(z) = \alpha / (1 - (1-\alpha) z^{-1})$, um único polo real em $z = 1 - \alpha$.

## 6. O compromisso: atenuação × atraso

Resposta calculada para $f_c = 20$ Hz (padrão do ArduCopter) e $f_s = 2$ kHz:

| f (Hz) | 2ª ordem: ganho | fase    | atraso de grupo | 1ª ordem: ganho | fase   |
|-------:|----------------:|--------:|----------------:|----------------:|-------:|
| 1      | 0,0 dB          | −4°     | 11,3 ms         | 0,0 dB          | −3°    |
| 5      | 0,0 dB          | −21°    | 11,9 ms         | −0,3 dB         | −14°   |
| 10     | −0,3 dB         | −43°    | 13,2 ms         | −1,0 dB         | −26°   |
| 20     | −3,0 dB         | −90°    | 11,3 ms         | −3,1 dB         | −44°   |
| 40     | −12,3 dB        | −137°   | 3,3 ms          | −7,2 dB         | −61°   |
| 80     | −24,2 dB        | −159°   | 0,8 ms          | −12,5 dB        | −69°   |
| 160    | −36,5 dB        | −170°   | 0,2 ms          | −18,3 dB        | −69°   |

O que a tabela mostra:

* O 2ª ordem cai **−12 dB/oitava** (−40 dB/década) contra −6 dB/oitava do 1ª ordem: a 160 Hz atenua
  ~67× contra ~8×. É por isso que ele é usado na IMU, onde o ruído é forte.
* Em baixa frequência ele se comporta como um **atraso puro** de aproximadamente
  $\tau \approx \sqrt2/(2\pi f_c) \approx 11$ ms. Esse atraso entra diretamente na malha de controle de
  taxa e consome margem de fase: a 10 Hz já são −43°.
* Por isso, subir `INS_GYRO_FILTER` deixa o controle mais "afiado" (permite ganhos maiores), e baixá-lo
  limpa o ruído, mas obriga a reduzir P e D. Quadricópteros pequenos (hélices de 3–5″), que têm dinâmica
  rápida e ruído em frequências mais altas, usam cortes de 60–100+ Hz; frames grandes ficam próximos de
  20–40 Hz. A tela *Initial Parameters* do Mission Planner sugere um valor a partir do diâmetro da hélice.

## 7. Outras coisas relevantes

**Notch harmônico vem antes do passa-baixas e é mais eficiente que ele.** O ruído de um multirotor é
concentrado na frequência de rotação dos motores e suas harmônicas. Um notch rastreando essa frequência
(`INS_HNTCH_MODE`: throttle, RPM via telemetria BDShot/ESC ou FFT embarcada) remove o pico com muito
menos atraso de fase na banda de controle do que baixar o passa-baixas. A prática recomendada é:
configure o notch primeiro, depois suba o `INS_GYRO_FILTER` o quanto o ruído residual permitir.
O notch também é um biquad derivado por transformação bilinear, com outra função de transferência
protótipo ($H(s) = (s^2 + \omega_0^2)/(s^2 + (\omega_0/Q)s + \omega_0^2)$).

**Os filtros dos PIDs.** Padrões típicos no Copter 4.x: `ATC_RAT_RLL/PIT_FLTD` e `FLTT` = 20 Hz,
`FLTE` = 0 (desligado); no yaw, `FLTE` = 2,5 Hz, porque o termo D de yaw quase sempre é zero e o erro
filtrado alimenta P e I. O filtro do D é crucial: derivar um sinal ruidoso multiplica o ruído por
$2\pi f$, então o D é sempre a primeira vítima de vibração.

**Acelerômetro.** `INS_ACCEL_FILTER` (padrão 20 Hz) usa o mesmo biquad. O EKF3 consome as amostras já
filtradas/integradas (delta-ângulos e delta-velocidades) e tem seu próprio tratamento de ruído, então
mexer nesse parâmetro afeta pouco a estimação de atitude, mas importa para o controle de
altitude/aceleração vertical.

**Anti-aliasing no sensor.** O ICM-42688-P tem um filtro anti-aliasing interno configurado pelo driver.
Nenhum filtro digital no FC consegue desfazer aliasing que já aconteceu no ADC — por isso montar a
controladora com isolamento de vibração (o 6C tem IMU com amortecimento interno) e balancear hélices
continua sendo a "primeira ordem" de qualquer projeto de filtros.

**Como medir em vez de chutar.** Habilite o *batch sampler* (`INS_LOG_BAT_MASK`, `INS_LOG_BAT_OPT`) para
logar o giroscópio na taxa crua, antes e depois dos filtros, e use a ferramenta de FFT do Mission
Planner ou o *Filter Review Tool* web do ArduPilot. Ela plota o espectro e a resposta dos filtros
configurados, incluindo o atraso de fase total — que é o número que realmente importa para o tuning.

**Custo computacional.** Cada biquad custa 5 multiplicações e 4 somas por amostra por eixo. A 8 kHz,
em três eixos e com vários notches, isso já é relevante; é uma das razões de o H7 do 6C (com FPU de precisão
simples) suportar *fast sampling* completo enquanto controladoras F4 antigas eram limitadas.

## 8. Resumo

* O `INS_GYRO_FILTER` do ArduPilot é um **Butterworth de 2ª ordem** discretizado por **transformação
  bilinear com pré-distorção**; os coeficientes do código saem diretamente de substituir
  $s/\omega_a = \frac{1}{K}\frac{1-z^{-1}}{1+z^{-1}}$, $K = \tan(\pi f_c/f_s)$, no protótipo
  $1/(s^2 + \sqrt2 s + 1)$.
* Os filtros dos PIDs são **RC de 1ª ordem** discretizados por **Euler implícito**,
  $\alpha = T/(T + RC)$.
* Na IMU o filtro roda na taxa de amostragem rápida (kHz), antes da subamostragem para o loop de
  400 Hz, o que evita aliasing do ruído de motor.
* Todo passa-baixas compra atenuação com atraso (~11 ms para o biquad a 20 Hz). O caminho moderno é
  remover os picos com notch harmônico e usar o passa-baixas mais alto possível.

---

*Referências:* código-fonte do ArduPilot (`libraries/Filter/LowPassFilter2p.cpp`,
`libraries/Filter/LowPassFilter.cpp`, `libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp`,
`libraries/AC_PID/AC_PID.cpp`); documentação ArduCopter — *Managing Gyro Noise with the Dynamic Harmonic
Notch Filters* e *Measuring Vibration with IMU Batch Sampler*; Oppenheim & Schafer, *Discrete-Time Signal
Processing*, cap. 7 (transformação bilinear). Os trechos de código acima são resumidos e podem diferir em
detalhes entre versões do firmware.
