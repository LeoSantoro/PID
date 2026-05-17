#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <string.h>
#include "SPIFFS.h"
#include <nvs_flash.h>
#include "WiFi.h"
#include <Arduino.h>

#define ESCPIN 5
#define SENSOR 4

// ---------------------------------Variaveis de controle----------------------------------
const int minPwm = 1015;                        // valor do PWM que mantém o motor no ponto zero a um passo de ligar
double setpoint = 0;                            // Definições da posição escolhida pelo usuário (setpoint)
double error = 0;                               // erro atual em relação ao setpoint
double angulo = 0, anguloAD = 0, lastErro = 0;  // ângulo atual e o último medido
double kp, ki, kd;                              // constantes do controlador PID
double P = 0, I = 0, D = 0, PID = 0;            // variáveis auxiliares do PID
double controle = 0;                            // valor de controle
String data;                                    // Dados que serão enviado à pagina
bool start = false;                             // Flag que indica inicialização do processo
long lastProcess = 0;                           // Armazena o tempo do último processo
long tempo = 0;                                 // Armazena o tempo para a váriavel data
int now = 0;                                    // Armazena o tempo atual do sistema
float deltaTime = 0;                            // Armazena a variação do tempo 
int calibracao[2];                              // Armazena os valores de calibração
int loopTime;                                   // Tempo de amostragem
uint16_t contador = 0;
double somatorio = 0;

// Criação da variavel de controle do ESC (PWM)
Servo esc;

// Cria o servidor na porta padrão 80
AsyncWebServer server(80);

// Cria a porta de evento
AsyncEventSource events("/events");

// Variavel para utilizar a memoria eeprom do ESP
nvs_handle handler_particao_nvs;

void grava_int_nvs(const char *nome, const char *local, int32_t dado)
{
  nvs_handle handler_particao_nvs;
  esp_err_t err;

  err = nvs_flash_init_partition("nvs");

  if (err != ESP_OK)
  {
    Serial.println("[ERRO] Falha ao iniciar partição NVS.");
    Serial.println(err);
    return;
  }

  err = nvs_open_from_partition("nvs", nome, NVS_READWRITE, &handler_particao_nvs);
  if (err != ESP_OK)
  {
    Serial.println("[ERRO] Falha ao abrir NVS como escrita/leitura");
    return;
  }

  /* Atualiza valor do horimetro total */
  err = nvs_set_i32(handler_particao_nvs, local, dado);

  if (err != ESP_OK)
  {
    Serial.println("[ERRO] Erro ao gravar int");
    Serial.println(err);
    return;
  }
  Serial.println("Dado gravado com sucesso!");
  nvs_commit(handler_particao_nvs);
  nvs_close(handler_particao_nvs);
}

int32_t le_int_nvs(const char *nome, const char *local)
{
  char buff[50];
  nvs_handle handler_particao_nvs;
  esp_err_t err;
  int32_t dado_lido;

  err = nvs_flash_init_partition("nvs");

  if (err != ESP_OK)
  {
    Serial.println("[ERRO] Falha ao iniciar partição NVS.");
    return -1;
  }

  err = nvs_open_from_partition("nvs", nome, NVS_READWRITE, &handler_particao_nvs);
  if (err != ESP_OK)
  {
    Serial.println("[ERRO] Falha ao abrir NVS como escrita/leitura");
    return -1;
  }

  /* Faz a leitura do dado associado a chave definida em CHAVE_NVS */
  err = nvs_get_i32(handler_particao_nvs, local, &dado_lido);

  if (err != ESP_OK)
  {
    sprintf(buff, "[ERRO] Dado não lido %s - %s", nome, local);
    Serial.println(buff);
    return -1;
  }
  else
  {
    sprintf(buff, "Dado lido %s - %s - %d", nome, local, dado_lido);
    Serial.println(buff);
    nvs_close(handler_particao_nvs);
    return dado_lido;
  }
}

// Função que le o ângulo atual
void leAngulo()
{
  anguloAD = analogRead(SENSOR);
  angulo = map(anguloAD, calibracao[0], calibracao[1], 0, 90);
}

void controleMotor(float tensao){  
  
  uint16_t controle = (uint16_t) (1000.0f + (tensao / 12.0f) * (2000.0f - 1000.0f));

  if (controle < 1000) controle = 1000;
  if (controle > 2000) controle = 2000;

  esc.writeMicroseconds(controle);
}

// Task que envia os dados ao servidor
void taskAtualiza(void *parameter)
{
  while (1)
  {
    if (start)
    {
      events.send(data.c_str(), "atualiza", NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup()
{
  Serial.begin(115200);

  analogReadResolution(12);      // Define resolução para 12 bits
  analogSetAttenuation(ADC_6db); // Define atenuação para 11dB (0-3.9V)

  // Inicialização do sistemas de arquivos
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Configuração do controle PWM
  esc.attach(ESCPIN);
  controleMotor(0);
  
  // Inicio da conexão WiFi
  WiFi.softAP("ESP32", NULL);

  // Definição do IP estático
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Rota para default
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  // Rota para página de calibração
  server.on("/calibracao", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/calibracao.html", "text/html"); });

  // Rota para calibração do ponto 0°
  server.on("/calibra0", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    calibracao[0] = analogRead(SENSOR);
    grava_int_nvs("valorAD", "0", calibracao[0]);
    events.send("Angulo 0° Calibrado!", "alert");
    request->send(200, "text/plain", "Calibrado"); });

  // Rota para a calibração do ponto 90°
  server.on("/calibra90", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    calibracao[1] = analogRead(SENSOR);
    grava_int_nvs("valorAD", "90", calibracao[1]);
    events.send("Angulo 90° Calibrado!", "alert");
    request->send(200, "text/plain", "Calibrado"); });

  // Rota para pagina de monitoramento
  server.on("/monitoramento", HTTP_GET, [](AsyncWebServerRequest *request)
            {

    setpoint = request->getParam(0)->value().toFloat();
    //setpoint = map(setpoint, 0, 90, calibracao[0], calibracao[1]);

    kp = request->getParam(1)->value().toFloat();
    ki = request->getParam(2)->value().toFloat();
    kd = request->getParam(3)->value().toFloat();

    request->send(SPIFFS, "/monitoramento.html", "text/html"); });

  // Rota para iniciar o monitoramento
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    P = I = D = PID = 0;
    request->send(200, "text/plain", "Iniciado");
    start = true;
    angulo = 0;
    lastProcess = 0;
    contador = 0;
    somatorio = 0;
    tempo = millis(); });

  // Rota para parar o monitoramento
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    esc.writeMicroseconds(1000);
    request->send(200, "text/plain", "Parado");
    start = false; });

  // Settando pathing para o servidor
  server.serveStatic("/", SPIFFS, "/");

  // Iniciando o SSE
  server.addHandler(&events);

  // Iniciando o servidor
  server.begin();

  // Criando task para enviar dados ao client
  xTaskCreatePinnedToCore(taskAtualiza,    /* Task function. */
                          "Task Atualiza", /* name of task. */
                          10000,           /* Stack size of task */
                          NULL,            /* parameter of the task */
                          1,               /* priority of the task */
                          NULL,            /* Task handle to keep track of created task */
                          0);              /* pin task to core 0 */

  // Recebe os dados da calibração
  calibracao[0] = le_int_nvs("valorAD", "0");
  calibracao[1] = le_int_nvs("valorAD", "90");
}

void loop()
{
  //Flag que indica que o sistem tem que iniciar
  while (start)
  {
    //Verifica se é o primeiro processo
    if (lastProcess == 0)
    {
      leAngulo();
      PID = I = D = P = 0;
      lastErro = setpoint;
      lastProcess = millis();
      now = millis();
      delay(1);
    }
    else
    {
      //calcula o delta tempo
      now = millis();
      deltaTime = (now - lastProcess) / 1000.0;
      lastProcess = now;

      if (contador > 0)
        angulo =(float)(somatorio / contador);
        
      contador = 0;
      somatorio = 0;

      //Calculo do erro
      error = setpoint - angulo;

      // Proporcional
      P = error * kp;

      // Integral
      I += (error * ki) * deltaTime;

      // Derivativo
      D = ((error - lastErro) * kd) / deltaTime;

      //Forma o controle PID
      PID = P + I + D;

      //Recebe a última leitura
      lastErro = error;
    }
    
    if (PID < 0) PID = 0;
    if (PID > 12) PID = 12;
    controleMotor(PID);

    data = "[" + String(millis() - tempo) + ", " + String(angulo) + "]";

    loopTime = millis() - now;

    while(loopTime < 50){
      leAngulo();
      somatorio += angulo;
      contador++;
      delay(1);
      loopTime = millis() - now;
    }  
  }
}
