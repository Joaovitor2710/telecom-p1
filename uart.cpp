#include "uart.hpp"
#include <array>

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n)
{
static enum { OCIOSO, INICIO, DADOS, PARADA } estado = OCIOSO;
static std::array<unsigned int, 30> janela_inicio = {};
static int indice_inicio = 0;
static int contador_amostras = 0;
static int indice_bit = 0;
static uint8_t byte_atual = 0;

for (unsigned int i = 0; i < n; ++i) {
    unsigned int amostra = buffer[i];

    switch (estado) {
        case OCIOSO:
            janela_inicio[indice_inicio] = amostra;
            indice_inicio = (indice_inicio + 1) % 30;
            if (amostra == 0) {
                int contagem_baixo = 0;
                for (int j = 0; j < 30; ++j)
                    if (janela_inicio[j] == 0)
                        ++contagem_baixo;

                if (contagem_baixo >= 25) {
                    estado = INICIO;
                    contador_amostras = 15;
                }
            }
            break;

        case INICIO:
            contador_amostras++;
            if (contador_amostras == 80) {
                contador_amostras = 0;
                indice_bit = 0;
                byte_atual = 0;
                estado = DADOS;
            }
            break;

        case DADOS:
            contador_amostras++;
            if (contador_amostras == 160) {
                contador_amostras = 0;

                byte_atual |= (amostra & 1) << indice_bit;
                indice_bit++;

                if (indice_bit == 8) {
                    estado = PARADA;
                }
            }
            break;

        case PARADA:
            contador_amostras++;
            if (contador_amostras == 160) {
                contador_amostras = 0;
                estado = OCIOSO;
                get_byte(byte_atual);  
            }
            break;
    }
}
}

void UART_TX::put_byte(uint8_t byte)
{
    samples_mutex.lock();
    put_bit(0);  
    for (int i = 0; i < 8; i++) {
        put_bit(byte & 1);
        byte >>= 1;
    }
    put_bit(1);  
    samples_mutex.unlock();
}

void UART_TX::get_samples(unsigned int *buffer, unsigned int n)
{
    samples_mutex.lock();
    std::vector<unsigned int>::size_type i = 0;
    while (!samples.empty() && i < n) {
        buffer[i++] = samples.front();
        samples.pop_front();
    }
    samples_mutex.unlock();

    while (i < n) {
        buffer[i++] = 1;
    }
}

void UART_TX::put_bit(unsigned int bit)
{
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++) {
        samples.push_back(bit);
    }
}
