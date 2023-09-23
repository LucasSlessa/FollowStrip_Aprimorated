#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

const int sensorEsquerdoPin = 17;  // Pino do sensor esquerdo
const int sensorDireitoPin = 16;   // Pino do sensor direito
const int sensorEsquerdoPin1 = 2;  // Pino do sensor esquerdo 1
const int sensorDireitoPin1 = 15;   // Pino do sensor direito 1
const int motorEsquerdoPino1 = 22; // Pino IN1 do motor esquerdo
const int motorEsquerdoPino2 = 21; // Pino IN2 do motor esquerdo
const int motorDireitoPino1 = 19;  // Pino IN3 do motor direito
const int motorDireitoPino2 = 18;  // Pino IN4 do motor direito

int direcaoAtual = 1; // 0: esquerda, 1: frente, 2: direita
int direcaoAnterior = -1; // Direção anterior, -1 indica nenhum caminho anterior




SemaphoreHandle_t mutex;

void avancar() {
    digitalWrite(motorEsquerdoPino1, HIGH);
    digitalWrite(motorEsquerdoPino2, LOW);
    digitalWrite(motorDireitoPino1, LOW);
    digitalWrite(motorDireitoPino2, HIGH);
}
void girarEsquerda() {
    digitalWrite(motorEsquerdoPino1, LOW);
    digitalWrite(motorEsquerdoPino2, HIGH);
    digitalWrite(motorDireitoPino1, LOW);
    digitalWrite(motorDireitoPino2, LOW);
}

void girarDireita() {
    digitalWrite(motorEsquerdoPino1, LOW);
    digitalWrite(motorEsquerdoPino2, LOW);
    digitalWrite(motorDireitoPino1, HIGH);
    digitalWrite(motorDireitoPino2, LOW);
}

void parar() {
    digitalWrite(motorEsquerdoPino1, LOW);
    digitalWrite(motorEsquerdoPino2, LOW);
    digitalWrite(motorDireitoPino1, LOW);
    digitalWrite(motorDireitoPino2, LOW);
}

void controlarSensores(void *param) {
    while (1) {
        // Leitura dos sensores
        int leituraSensorEsquerdo = digitalRead(sensorEsquerdoPin);
        int leituraSensorDireito = digitalRead(sensorDireitoPin);
        int leituraSensorEsquerdo1 = digitalRead(sensorEsquerdoPin1);
        int leituraSensorDireito1 = digitalRead(sensorDireitoPin1);

        xSemaphoreTake(mutex, portMAX_DELAY);
    // Lógica de controle dos sensores
    if (leituraSensorEsquerdo == HIGH && leituraSensorDireito == HIGH) {
 
    // Verifique se os sensores adicionais também não detectam a faixa
    if (leituraSensorEsquerdo1 == HIGH && leituraSensorDireito1 == HIGH) {
        // Nenhum sensor detecta a faixa, gire 180 graus e escolha uma direção alternativa

        if (direcaoAnterior == 0) {
            girarDireita();
            delay(1000); // tempo de giro
        } else if (direcaoAnterior == 2) {
            girarEsquerda();
            delay(1000); // tempo de giro
        }

        int direcaoAlternativa;
        do {
            direcaoAlternativa = random(0, 3); // 0: esquerda, 1: frente, 2: direita
        } while (direcaoAlternativa == direcaoAnterior);

        direcaoAnterior = direcaoAtual;
        direcaoAtual = direcaoAlternativa;
    } else {
        // Apenas os sensores principais não detectam a faixa, siga em frente

        avancar();
    }
      } else if (leituraSensorEsquerdo == LOW && leituraSensorDireito == LOW) {
           // Avançar Ambos os sensores estão na linha
           avancar();
      } else if (leituraSensorEsquerdo == HIGH && leituraSensorDireito == LOW) {
          // Desvio para a direita.
           girarDireita();
      } else if (leituraSensorEsquerdo == LOW && leituraSensorDireito == HIGH) {
          // Desvio para a esquerda.
          girarEsquerda();
      } else if (leituraSensorEsquerdo == HIGH && leituraSensorDireito == HIGH && leituraSensorEsquerdo1 == LOW && leituraSensorDireito1== HIGH) {
           // Desvio para a esquerda.
          girarEsquerda(); 
      } else if (leituraSensorEsquerdo == HIGH && leituraSensorDireito == HIGH && leituraSensorEsquerdo1 == HIGH && leituraSensorDireito1== LOW) {
          // Desvio para a direita.
          girarDireita();
          
      } else {
           // Ambos os sensores principais detectam a faixa, siga em frente.
           avancar();
}

        xSemaphoreGive(mutex);

        delay(10);
    }
}

void controlarMotores(void *param) {
    while (1) {
       
        xSemaphoreTake(mutex, portMAX_DELAY);

        if (direcaoAtual == 0) {
            // Girar para a esquerda.
            girarEsquerda();
        } else if (direcaoAtual == 1) {
            // Seguir em frente.
            avancar();
        } else if (direcaoAtual == 2) {
            // Girar para a direita.
            girarDireita();
        }

       
        xSemaphoreGive(mutex);

        delay(10);
    }
}

void setup() {
    pinMode(sensorEsquerdoPin, INPUT);
    pinMode(sensorDireitoPin, INPUT);
    pinMode(sensorEsquerdoPin1, INPUT);
    pinMode(sensorDireitoPin1, INPUT);
    pinMode(motorEsquerdoPino1, OUTPUT);
    pinMode(motorEsquerdoPino2, OUTPUT);
    pinMode(motorDireitoPino1, OUTPUT);
    pinMode(motorDireitoPino2, OUTPUT);


    // Inicialize o mutex
    mutex = xSemaphoreCreateMutex();

    // Crie tarefas para controlar sensores e motores nos dois CPUs do ESP32
    xTaskCreatePinnedToCore(controlarSensores, "SensorTask", 4096, NULL, 1, NULL, 1); // CPU 1
    xTaskCreatePinnedToCore(controlarMotores, "MotorTask", 4096, NULL, 1, NULL, 0); // CPU 0
}

void loop() {}

  

