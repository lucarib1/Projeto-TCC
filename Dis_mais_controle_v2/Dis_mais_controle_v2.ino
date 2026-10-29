#include <ESP32Encoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <DHT.h>
//===========Pinos_Encoder===========================
#define PINO_CLK 2
#define PINO_DT  4
#define PINO_SW  5
//===========Definição_Display_Oled==================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
//===========Pinos_controle_de_Temperatura===========
#define DHTPIN 15
#define DHTTYPE DHT11
#define PINO_VENTILADOR 18  // GPIO que controla o relé da pastilha peltier


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);//função que chama o display
DHT dht(DHTPIN, DHTTYPE); //função que chama o sensor DHT

//=========================
ESP32Encoder encoder;

float passoencoder = 0;
float posicao = 1;
long posicaoAnterior = 0;
long posicaoAtual = 0;
//=========================
float selecionarMenu = 0;
float tensaoCorte = 2.80;
char cursor = '>';
//==========Variaveis do controle de corrente e tensao===============
float setpoint = 1.0;
float input = 0;
float output = 0;  //Entrada e saída do PID
float multiplicador_tensao = 0;    // Utilizado para leitura de tensão
float multiplicador_corrente = 0;  // Utilizado para leitura de corrente
float tensao = 0;
float corrente = 0;
float corrente_set = 1;
double erro;
Adafruit_ADS1115 ads;  // Inicializa o ADS
unsigned long tempo_anterior = 0;  // Guarda o milissegundo da leitura anterior
unsigned long tempo_inicio = 0;    // Guarda o milissegundo de início do teste
float capacidade_mah = 0.0;        // Acumulador da capacidade da bateria em mAh
//=========================
unsigned long tempo_total_ms;
unsigned long total_segundos;
unsigned long horas;
unsigned long minutos;
unsigned long segundos;
unsigned long tempoEspera;
bool testeIniciado;
int saidaMenu = 0;

float temperatura = 0; //Temperatura lida pelo sensor DHT
//==========Variaveis do tempo para a aquisição de temperatura===============
unsigned long tempoAnteriorDHT = 0;
const unsigned long intervaloDHT = 2000;
const float temperaturaLiga = 20;
const float histerese = 1.0;
bool ventiladorLigado = false;


// ===== Aquisição de dados a cada 10 segundos =====
unsigned long tempoUltimaAquisicao = 0;
const unsigned long intervaloAquisicao = 10000;
int cabecalho = 0; // função para o cabeçalho do monitonitor serial 
unsigned long tempoAquisicaoSegundos = 0;


void setup() {
  pinMode(25, OUTPUT);
  pinMode(32, INPUT);
  pinMode(12, INPUT);
  pinMode(19, OUTPUT);
  Serial.begin(115200);
  digitalWrite(19, HIGH);

  Wire.begin(21, 22);  // SDA, SCL
  analogSetAttenuation(ADC_11db);
  Serial.begin(115200);

  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED nao encontrado!");
    while (true)
      ;
  }

  if (!ads.begin()) {
    Serial.println("ADS1115 não encontrado!");
    while (1)
      ;
  }

  telaInicial();
  pinMode(PINO_SW, INPUT_PULLUP);



  // Habilita os pull-ups internos
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Utiliza os dois canais do encoder (Quadrature)
  encoder.attachHalfQuad(PINO_DT, PINO_CLK);

  // Zera o contador
  encoder.setCount(0);

//Inicia os pinos para o controle de temperatura
  pinMode(PINO_VENTILADOR, OUTPUT);
  dht.begin();
//===========================================
  delay(2000);
}

void loop() {

  if (selecionarMenu == 0){
    digitalWrite(19, HIGH);
    telaMenu();
  } 
  else if (selecionarMenu == 1){
    telaTensao();
  } 
  else if (selecionarMenu == 2) {
    telaCorrente();
  }
  else if (selecionarMenu == 3) {
    digitalWrite(19, LOW);
    telaTeste();
    tensaoecorrente();
    ControleAdaptativo();
    calcularCapacidade();
    lerDHT();
    adquirirDados();


  }
  else if (selecionarMenu == 4) {
    digitalWrite(19, HIGH);
    testeFinalizado();
}
}

void tensaoecorrente() {
  float multiplicador_tensao = 0.000189;  // Por padrão  ADS1115  0.1875mV  @ +/- 6.144V gain (16-bit results)
  /* tensao = ads.readADC_SingleEnded(0);    //Read voltage
  tensao = (tensao * multiplicador_tensao);*/
  long soma = 0;

  for (int i = 0; i < 10; i++) {
    soma += ads.readADC_SingleEnded(0);
  }

  float leitura = soma / 10.0;

  tensao = leitura * multiplicador_tensao;
 //Serial.print("  Tensao: ");
  //Serial.println(tensao, 2);



  float multiplicador_corrente = (0.000189 / 0.5);  //O 0,5 e da resstencia do ressitor da carga
  corrente = ads.readADC_Differential_2_3();        //Read DIFFERENTIAL voltage between ADC0 and ADC1. (the load is 1ohm, so this is equal to the current)
  corrente = (corrente * multiplicador_corrente);
  //Serial.print("  Corrente: ");
  //Serial.println(corrente, 2);
}
//========================================================
void ControleAdaptativo() {
  setpoint = corrente_set;
  input = corrente;

  erro = setpoint - input;

  int passo = 0;

  // Define o tamanho do passo


  if (fabs(erro) > 0.8) {
    passo = 150;
  }

  else if (fabs(erro) > 0.6) {
    passo = 100;
  }

  else if (fabs(erro) > 0.4) {
    passo = 30;
  }

  else if (fabs(erro) > 0.3) {
    passo = 20;
  }

  else if (fabs(erro) > 0.2) {
    passo = 10;
  }

  else if (fabs(erro) > 0.01) {
    passo = 1;
  }


  else {
    passo = 0;
  }


  // Aumenta ou diminui o DAC
  if (erro > 0)
    output += passo;

  else if (erro < 0)
    output -= passo;

  // Limita entre 0 e 255
  output = constrain(output, 0, 255);

  // Envia para o DAC
  analogWrite(25, output);

  //Serial.print("Corrente: ");
 // Serial.print(corrente, 2);

  //Serial.print("  Erro: ");
  //Serial.print(erro, 3);

 //Serial.print("  Passo: ");
  //Serial.print(passo);

  //Serial.print("  DAC: ");
 // Serial.println(output);
  delay(100);
}
//========================================================
void calcularCapacidade() {
  unsigned long tempo_atual = millis();
  unsigned long dt_ms = tempo_atual - tempo_anterior;
  tempo_anterior = tempo_atual;
  // Converte a corrente de Amperes para Miliamperes (mA)
  float corrente_ma = corrente * 1000.0;
  // Só calcula se a bateria estiver de fato descarregando (corrente > 10mA)
  if (corrente > 0.01) {
    // dt_ms / 3600000.0 converte milissegundos em fração de hora
    float delta_mah = corrente_ma * ((float)dt_ms / 3600000.0);
    capacidade_mah += delta_mah;
  }
  // Calcula o tempo total decorrido do teste
  tempo_total_ms = tempo_atual - tempo_inicio;
  total_segundos = tempo_total_ms / 1000;
  horas = total_segundos / 3600;
  minutos = (total_segundos % 3600) / 60;
  segundos = total_segundos % 60;
  // Exibe os dados formatados
  /*
  Serial.print("  Tempo: ");
  if (horas < 10) Serial.print("0");
  Serial.print(horas);
  Serial.print(":");
  if (minutos < 10) Serial.print("0");
  Serial.print(minutos);
  Serial.print(":");
  if (segundos < 10) Serial.print("0");
  Serial.print(segundos);
  Serial.print("  Capacidade: ");
  Serial.print(capacidade_mah, 2);
  Serial.println(" mAh");
*/
  if(tensao < tensaoCorte){
    selecionarMenu = 4;
  }
}

//========================================================


void telaInicial() {

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);

  display.setCursor(43, 6);
  display.println("Medidor");

  display.setCursor(34, 20);
  display.println("Capacidade");

  display.setCursor(31, 34);
  display.println("de Baterias");

  display.display();
  delay(5000);
}

//========================================================

void telaTeste() {

  if (posicao == 3 && !testeIniciado) {
    tempo_inicio = millis();        // Funções para zerar o tempo no inicio do teste
    tempo_anterior = tempo_inicio;  //
    testeIniciado = true;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("TESTE EM ANDAMENTO");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 18);
  display.print("Vcorte:");
  
  display.setTextSize(1);
  display.setCursor(45, 18);
  display.print(tensaoCorte, 2);
  display.print("V ");
  display.print(temperatura,1);
  display.write(247);  // símbolo °
  display.print("C");

  display.setCursor(0, 28);
  display.print("Vbat:");

  display.setTextSize(1);
  display.setCursor(45, 28);
  display.print(tensao, 2);

  display.setTextSize(1);
  display.print("V");

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print("I:");

  display.setTextSize(1);
  display.setCursor(18, 40);
  display.print(corrente, 2);

  display.setTextSize(1);
  display.print("A");

  display.setCursor(0, 50);
  display.print("Cap:");

  display.setCursor(22, 50);
  display.print(capacidade_mah);
  display.print("mAh");



  display.setCursor(78, 50);


  if (horas < 10) { display.print("0"); }
  display.print(horas);
  display.print(":");

  if (minutos < 10) { display.print("0"); }
  display.print(minutos);
  display.print(":");

  if (segundos < 10) { display.print("0"); }
  display.print(segundos);

  display.display();


  if (digitalRead(PINO_SW) == LOW) {
    saidaMenu = 1;
    delay(500);
  }
  while (saidaMenu == 1) {
    passoencoder = 1;
    posicaoAtual = encoder.getCount();

    // Atualiza somente quando houver mudança
    if (posicaoAtual != posicaoAnterior) {
      delay(200);
      if (posicaoAtual > posicaoAnterior) {
        posicao = posicao + passoencoder;
      } else {
        posicao = posicao - passoencoder;
      }

      posicaoAnterior = posicaoAtual;
    }
    if (posicao > 1) {
      posicao = 0;
    }
    if (posicao < 0) {
      posicao = 1;
    }
    if (posicao == 0) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(23, 8);
      display.print("Parar o teste?");
      display.drawLine(0, 18, 128, 18, SSD1306_WHITE);
      display.setCursor(0, 30);
      display.print(cursor);
      display.setCursor(6, 30);
      display.print(" SIM");
      display.setCursor(1, 45);
      display.print("  NAO");
      display.display();

      if (digitalRead(PINO_SW) == LOW) {
        saidaMenu = 0;
        selecionarMenu = 4;
        delay(500);
      }
    }
    if (posicao == 1) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(23, 8);
      display.print("Parar o teste?");
      display.drawLine(0, 18, 128, 18, SSD1306_WHITE);
      display.setCursor(6, 30);
      display.print(" SIM");
      display.setCursor(0, 45);
      display.print(cursor);
      display.setCursor(6, 45);
      display.print(" NAO");
      display.display();

      if (digitalRead(PINO_SW) == LOW) {
        saidaMenu = 0;
        selecionarMenu = 3;
        delay(500);
      }
    }
  }
}
void telaTensao() {

  posicaoAtual = encoder.getCount();

  if (posicaoAtual != posicaoAnterior) {
    delay(200);
    // Serial.print("Posicao: ");
    // Serial.println(posicaoAtual);
    if (posicaoAtual > posicaoAnterior) {
      tensaoCorte = tensaoCorte + 0.05;  // incremento de 0,1 no valor da corrente
      //Serial.println("tensaoCorte");
      //Serial.println(tensaoCorte);
    } else {
      tensaoCorte = tensaoCorte - 0.05;  // decremento de 0,1 no valor da corrente
      //Serial.println(tensaoCorte);
    }
    posicaoAnterior = posicaoAtual;
  }

  if (tensaoCorte > 4.2) {
    tensaoCorte = 4.2;
  }
  if (tensaoCorte < 0) {
    tensaoCorte = 0;
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TENSAO DE CORTE");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  //display.setTextSize(2);
  // display.setCursor(16, 14);
  //display.print("Tensao");

  // Valor da corrente
  display.setTextSize(2);
  display.setCursor(28, 35);
  display.print(tensaoCorte, 2);
  display.setCursor(80, 35);
  display.print("V");
  display.display();
  if (digitalRead(PINO_SW) == LOW) {
    selecionarMenu = 0;
    delay(500);
  }
}
void telaCorrente() {
  passoencoder = 0.1;
  posicaoAtual = encoder.getCount();

  if (posicaoAtual != posicaoAnterior) {
    delay(200);
   // Serial.print("Posicao: ");
   // Serial.println(posicaoAtual);
    if (posicaoAtual > posicaoAnterior) {
      corrente_set = corrente_set + 0.1; // incremento de 0,1 no valor da corrente
      Serial.println("corrente");
      Serial.println(corrente_set);
    } else {
      corrente_set = corrente_set - 0.1; // decremento de 0,1 no valor da corrente
      Serial.println(corrente_set);
    }
    posicaoAnterior = posicaoAtual;
  }
  
  if (corrente_set > 2.5) {
    corrente_set = 2.5;
  }
  if (corrente_set < 0) {
    corrente_set = 0;
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("CORRENTE");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // Valor da corrente
  display.setTextSize(2);
  display.setCursor(28, 40);
  display.print(corrente_set);
  display.setCursor(80, 40);
  display.print("A");
  display.display();

 if (digitalRead(PINO_SW) == LOW) {
  selecionarMenu = 0;
  delay(500);
  }
}
void telaMenu(){
testeIniciado = false; //determina o ponto do cronometro do tempo
passoencoder = 1;
posicaoAtual = encoder.getCount();

  // Atualiza somente quando houver mudança
  if (posicaoAtual != posicaoAnterior) {
    delay(200);
   // Serial.print("Posicao: ");
   // Serial.println(posicaoAtual);
    if (posicaoAtual > posicaoAnterior) {
      posicao = posicao + passoencoder;
      Serial.println(posicao);
    } else {
      posicao = posicao - passoencoder;
      Serial.println(posicao);


    }
    // Serial.print("Posicao: ");
    //Serial.println(posicaoAtual);
    posicaoAnterior = posicaoAtual;
  }
  
  // Leitura do botão
  if (digitalRead(PINO_SW) == LOW) {
    selecionarMenu = posicao;
    Serial.println("Botao pressionado");
    delay(200);  // debounce do botão
  }

if (posicao > 3) {
  posicao = 1;

} 
if (posicao <= 0) {
  posicao = 3;
  }

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  // Título

  display.setTextSize(1);

  display.setCursor(0,0);
  display.println("MENU");

  display.drawLine(0,10,128,10,SSD1306_WHITE);

  // Itens
if (posicao == 1) {
  display.setCursor(0,18);
  display.print(cursor);
  display.setCursor(6,18);
  display.print("Tensao de Corte");
  display.setCursor(100,18);
  display.print(tensaoCorte,2);

  display.setCursor(0,34);
  //display.print(cursor);
  display.setCursor(1,34);
  display.print(" Corrente");
  display.setCursor(100,34);
  display.print(corrente_set,2);

  //display.setCursor(0,50);
 //display.print(cursor);
  display.setCursor(1,50);
  display.print(" Iniciar Teste");
}
else if (posicao == 2) {
  //display.setCursor(0,18);
 // display.print(cursor);
  display.setCursor(6,18);
  display.print("Tensao de Corte");
  display.setCursor(100,18);
  display.print(tensaoCorte,2);

  display.setCursor(0,34);
  display.print(cursor);
  display.setCursor(1,34);
  display.print(" Corrente");
  display.setCursor(100,34);
  display.print(corrente_set,2);

  //display.setCursor(0,50);
  //display.print(cursor);
  display.setCursor(1,50);
  display.print(" Iniciar Teste");
}
else if (posicao == 3 ){
  display.setCursor(0,18);
  //display.print(cursor);
  display.setCursor(6,18);
  display.print("Tensao de Corte");
  display.setCursor(100,18);
  display.print(tensaoCorte,2);

  // display.setCursor(0,34);
  //display.print(cursor);
  display.setCursor(1,34);
  display.print(" Corrente");
  display.setCursor(100,34);
  display.print(corrente_set,2);

  display.setCursor(0,50);
  display.print(cursor);
  display.setCursor(1,50);
  display.print(" Iniciar Teste");
}
  display.display();

}
void testeFinalizado(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(10, 0);
  display.print("TESTE EM FINALIZADO");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(12, 18);
  display.print("CAPACIDADE:");

  display.setCursor(15, 30);
  display.print(capacidade_mah, 1);
  display.print("mAh");

  display.setCursor(12, 43);
  display.print("TEMPO:");
  display.setCursor(12, 55);

  if (horas < 10) { display.print("0"); }
  display.print(horas);
  display.print(":");

  if (minutos < 10) { display.print("0"); }
  display.print(minutos);
  display.print(":");

  if (segundos < 10) { display.print("0"); }
  display.print(segundos);

  display.display();


  if (digitalRead(PINO_SW) == LOW) {
    selecionarMenu = 0;
    capacidade_mah = 0;
    delay(500);
}
}

void lerDHT() {
  if (millis() - tempoAnteriorDHT >= intervaloDHT) {
    tempoAnteriorDHT = millis();

    float t = dht.readTemperature();

    temperatura = t;

    if (!ventiladorLigado && temperatura >= temperaturaLiga) {
    ventiladorLigado = true;
    digitalWrite(PINO_VENTILADOR, HIGH);

  }

  // Desliga apenas quando cair para 19°C
  else if (ventiladorLigado && temperatura <= (temperaturaLiga - histerese)) {

    ventiladorLigado = false;
    digitalWrite(PINO_VENTILADOR, LOW);

  }
  }
  // Liga quando atingir 20°C
  
}
void adquirirDados() {

  unsigned long tempoAtual = millis();
  
if (cabecalho < 1) {
  // Exibe os dados no Monitor Serial
  Serial.println("===== AQUISICAO DE DADOS =====");
  Serial.println("Tempo_s,Tensao_V,Corrente_A,Capacidade_mAh");
  cabecalho = 1;
}

  // Verifica se passaram 15 segundos
  if (tempoAtual - tempoUltimaAquisicao >= intervaloAquisicao) {

    tempoUltimaAquisicao = tempoAtual;

    // Tempo decorrido desde o início do teste
    tempoAquisicaoSegundos = (tempoAtual - tempo_inicio) / 1000;

    // Exibe os dados no Monitor Serial
   
    // Dados separados por vírgula
    Serial.print(tempo_total_ms / 1000);
    Serial.print(",");

    Serial.print(tensao, 3);
    Serial.print(",");

    Serial.print(corrente, 3);
    Serial.print(",");

    Serial.println(capacidade_mah, 2);

  }
}