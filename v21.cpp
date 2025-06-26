#include <math.h>
#include <numbers>
#include <stdio.h>
#include "v21.hpp"


//Utilizando o código offline passado pelo professor em https://colab.research.google.com/drive/1tjileevEYGz6IMGCzqgFq9Sy4GvYtBuL?usp=sharing#scrollTo=obj1JdrdOBOt
//apenas passando pra online e consertando alguns detalhes pra funcionar.
void V21_RX::demodulate(const float *in, unsigned int n)
{

    unsigned int digital_samples[n];

    const int L = SAMPLES_PER_SYMBOL;
    const float T = SAMPLING_PERIOD;
    float buf[L + 1] = {};
    int idx = 0, cnt = 0;

    float r = 0.99f;

     /*
    from scipy.signal import butter

    fs = 48000
    fc = 300
    b, a = butter(2, fc/(fs/2), btype='low', analog=False)

    print('b =', b)
    print('a =', a)

    saída:
    b = [0.00037507 0.00075014 0.00037507]
    a = [ 1.         -1.94447766  0.94597794]
    */

    constexpr float b0 = 0.00037507f;
    constexpr float b1 = 0.00075014f;
    constexpr float b2 = 0.00037507f;

    constexpr float a1 = -1.94447766f;
    constexpr float a2 = 0.94597794f;

    for (unsigned int i = 0; i < n; i++) {
        buf[idx] = in[i];
        idx = (idx + 1) % (L + 1);
        float x = buf[idx]; 

        float c0 = std::pow(r, L) * std::cos(omega_space * L * T);
        float s0 = std::pow(r, L) * std::sin(omega_space * L * T);
        float c1 = std::cos(omega_space * T);
        float s1 = std::sin(omega_space * T);

        float c2 = std::pow(r, L) * std::cos(omega_mark * L * T);
        float s2 = std::pow(r, L) * std::sin(omega_mark * L * T);
        float c3 = std::cos(omega_mark * T);
        float s3 = std::sin(omega_mark * T);

        float p0 = in[i] - c0 * x + r * c1 * vr0 - r * s1 * vi0;
        float q0 = -s0 * x + r * c1 * vi0 + r * s1 * vr0;

        float p1 = in[i] - c2 * x + r * c3 * vr1 - r * s3 * vi1;
        float q1 = -s2 * x + r * c3 * vi1 + r * s3 * vr1;

        vr0 = p0; vi0 = q0;
        vr1 = p1; vi1 = q1;

        float y = (p1 * p1 + q1 * q1) - (p0 * p0 + q0 * q0);
        float f = b0 * y + b1 * d0 + b2 * d1 - a1 * f0 - a2 * f1;

        d1 = d0;
        d0 = y;
        f1 = f0;
        f0 = f;

        if (abs(f) > 60.0f)
            cnt = 50;
        else if (cnt > 0 && abs(f) < 50.0f)
            cnt--;

        digital_samples[i] = (cnt > 0) ? (f >= 0.0f) : 1;
    }

    get_digital_samples(digital_samples, n);

}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n)
{
    while (n--) {
        *out_analog_samples++ = sin(phase);
        phase += (*in_digital_samples++ ? omega_mark : omega_space) * SAMPLING_PERIOD;

        // evita que phase cresça indefinidamente, o que causaria perda de precisão
        phase = remainder(phase, 2*std::numbers::pi);
    }
}
