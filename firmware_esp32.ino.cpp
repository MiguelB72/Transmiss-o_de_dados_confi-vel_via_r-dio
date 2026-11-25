#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PINO_TX   33
#define PINO_RX   32
#define PINO_SDA  26
#define PINO_SCL  27

#define MEIO_BIT_US   500
#define SYNC_WORD     0x2DD4

#define FLAG_INICIO       0x7E
#define MAX_PAYLOAD       16
#define MAX_BUFFER_DADOS  4096
#define TIMEOUT_ACK_MS    700
#define MAX_TENTATIVAS    10

enum TipoQuadro : uint8_t {
  TIPO_DATA = 0x01,
  TIPO_ACK  = 0x02,
  TIPO_END  = 0x03
};

enum TipoConteudo : uint8_t {
  CONTEUDO_TEXTO  = 0x01,
  CONTEUDO_IMAGEM = 0x02
};

struct Quadro {
  uint8_t flag;
  uint8_t type;
  uint8_t seq;
  uint8_t len;
  uint8_t data[MAX_PAYLOAD];
  uint8_t fcs;
};

uint8_t bufferTransmissao[MAX_BUFFER_DADOS];
uint8_t bufferRecepcao[MAX_BUFFER_DADOS];
uint16_t tamanhoRecebido = 0;

LiquidCrystal_I2C* lcd = nullptr;

bool emSessao = false;
uint8_t seqEsperado = 0;
int16_t ultimoSeqEnd = -1;
int ultimoEstadoRX = LOW;

void inicializarLCD() {
  Wire.begin(PINO_SDA, PINO_SCL);
  uint8_t enderecos[] = {0x27, 0x3F};

  for (uint8_t addr : enderecos) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      lcd = new LiquidCrystal_I2C(addr, 16, 2);
      lcd->init();
      lcd->backlight();
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("RF 433 MHz Pronto");
      lcd->setCursor(0, 1);
      lcd->print("Aguardando...");
      Serial.printf("[LCD] Display detectado no endereco 0x%02X\n", addr);
      return;
    }
  }
  Serial.println("[LCD] Nenhum display detectado nesta placa.");
}

void exibirNoLCD(const String &linha1, const String &linha2 = "") {
  if (lcd == nullptr) return;

  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print(linha1.substring(0, min((int)linha1.length(), 16)));

  if (linha2.length() > 0) {
    lcd->setCursor(0, 1);
    lcd->print(linha2.substring(0, min((int)linha2.length(), 16)));
  }
}

void exibirTextoRecebidoLCD(const uint8_t* dados, uint16_t tamanho) {
  String texto = "";
  uint16_t limite = min((uint16_t)32, tamanho);
  for (uint16_t i = 0; i < limite; i++) {
    texto += (char)dados[i];
  }

  String l1 = texto.substring(0, min((int)texto.length(), 16));
  String l2 = (texto.length() > 16) ? texto.substring(16, min((int)texto.length(), 32)) : "";
  exibirNoLCD(l1, l2);
}

uint8_t calcularFCS(const Quadro &q) {
  uint8_t soma = 0;
  soma += q.type;
  soma += q.seq;
  soma += q.len;

  for (uint8_t i = 0; i < q.len; i++) {
    soma += q.data[i];
  }

  return ~soma;
}

void enviarBitManchester(bool bitVal) {
  if (bitVal) {
    digitalWrite(PINO_TX, HIGH);
    delayMicroseconds(MEIO_BIT_US);
    digitalWrite(PINO_TX, LOW);
    delayMicroseconds(MEIO_BIT_US);
  } else {
    digitalWrite(PINO_TX, LOW);
    delayMicroseconds(MEIO_BIT_US);
    digitalWrite(PINO_TX, HIGH);
    delayMicroseconds(MEIO_BIT_US);
  }
}

void enviarByteRF(uint8_t b) {
  for (int8_t i = 7; i >= 0; i--) {
    enviarBitManchester((b >> i) & 0x01);
  }
}

void enviarQuadroRF(Quadro &q, bool simularErro = false) {
  q.flag = FLAG_INICIO;
  q.fcs = calcularFCS(q);

  uint8_t buffer[21];
  uint8_t idx = 0;
  buffer[idx++] = q.flag;
  buffer[idx++] = q.type;
  buffer[idx++] = q.seq;
  buffer[idx++] = q.len;

  for (uint8_t i = 0; i < q.len; i++) {
    buffer[idx++] = q.data[i];
  }
  buffer[idx++] = q.fcs;

  if (simularErro) {
    if (q.len > 0) {
      buffer[4] ^= 0x01;
    } else {
      buffer[2] ^= 0x01;
    }
    Serial.println("[SIMULACAO] Bit invertido propositalmente nesta tentativa!");
  }

  for (uint8_t i = 0; i < 4; i++) {
    enviarByteRF(0xAA);
  }

  enviarByteRF((SYNC_WORD >> 8) & 0xFF);
  enviarByteRF(SYNC_WORD & 0xFF);

  for (uint8_t i = 0; i < idx; i++) {
    enviarByteRF(buffer[i]);
  }

  digitalWrite(PINO_TX, LOW);
  ultimoEstadoRX = digitalRead(PINO_RX);
}

bool aguardarTransicao(uint32_t timeoutUs) {
  int estadoInicial = digitalRead(PINO_RX);
  uint32_t inicio = micros();

  while (digitalRead(PINO_RX) == estadoInicial) {
    if ((micros() - inicio) > timeoutUs) {
      return false;
    }
  }
  return true;
}

bool lerBitManchester(bool &bitLido) {
  delayMicroseconds(MEIO_BIT_US + (MEIO_BIT_US / 2));
  bitLido = (digitalRead(PINO_RX) == HIGH);
  return aguardarTransicao(MEIO_BIT_US * 2);
}

bool lerByteRF(uint8_t &byteLido) {
  byteLido = 0;
  for (uint8_t i = 0; i < 8; i++) {
    bool bitVal = false;
    if (!lerBitManchester(bitVal)) {
      return false;
    }
    byteLido = (byteLido << 1) | (bitVal ? 1 : 0);
  }
  return true;
}

bool receberQuadroRF(Quadro &q) {
  int estadoAtual = digitalRead(PINO_RX);
  if (estadoAtual == ultimoEstadoRX) {
    return false;
  }
  ultimoEstadoRX = estadoAtual;

  uint16_t shiftSync = 0;
  bool sincronizado = false;

  for (uint8_t tentativas = 0; tentativas < 48; tentativas++) {
    bool bitVal = false;
    if (!lerBitManchester(bitVal)) {
      ultimoEstadoRX = digitalRead(PINO_RX);
      return false;
    }
    shiftSync = (shiftSync << 1) | (bitVal ? 1 : 0);
    if (shiftSync == SYNC_WORD) {
      sincronizado = true;
      break;
    }
  }

  if (!sincronizado) {
    ultimoEstadoRX = digitalRead(PINO_RX);
    return false;
  }

  if (!lerByteRF(q.flag) || q.flag != FLAG_INICIO) return false;
  if (!lerByteRF(q.type)) return false;
  if (!lerByteRF(q.seq))  return false;
  if (!lerByteRF(q.len) || q.len > MAX_PAYLOAD) return false;

  for (uint8_t i = 0; i < q.len; i++) {
    if (!lerByteRF(q.data[i])) return false;
  }

  if (!lerByteRF(q.fcs)) return false;

  ultimoEstadoRX = digitalRead(PINO_RX);
  return true;
}

bool enviarQuadroConfiavel(Quadro &q, bool simularErroNaPrimeira = false) {
  for (uint8_t tentativa = 1; tentativa <= MAX_TENTATIVAS; tentativa++) {
    bool injetarFalha = (simularErroNaPrimeira && tentativa == 1);

    if (tentativa == 1) {
      Serial.printf("[TX_%s] SEQ=%d | LEN=%d | Tentativa 1\n",
                    (q.type == TIPO_END) ? "END" : "DATA", q.seq, q.len);
    } else {
      Serial.printf("[RETRANSMIT] SEQ=%d | Tentativa %d de %d\n",
                    q.seq, tentativa, MAX_TENTATIVAS);
    }

    enviarQuadroRF(q, injetarFalha);

    uint32_t inicioEspera = millis();
    while ((millis() - inicioEspera) < TIMEOUT_ACK_MS) {
      Quadro resp;
      if (receberQuadroRF(resp)) {
        if (calcularFCS(resp) == resp.fcs &&
            resp.type == TIPO_ACK &&
            resp.seq == q.seq) {
          Serial.printf("[ACK_OK] Confirmacao recebida para SEQ=%d\n", q.seq);
          return true;
        }
      }
    }

    Serial.printf("[TIMEOUT] Sem ACK para SEQ=%d apos %d ms\n", q.seq, TIMEOUT_ACK_MS);
  }

  Serial.printf("[FALHA_CRITICA] Limite de tentativas atingido para SEQ=%d\n", q.seq);
  return false;
}

void transmitirBuffer(const uint8_t* dados, uint16_t totalBytes, uint8_t tipoConteudo, bool simularErro) {
  uint16_t offset = 0;
  uint8_t seq = 0;
  uint16_t totalQuadros = (totalBytes + MAX_PAYLOAD - 1) / MAX_PAYLOAD;

  Serial.printf("\n--- Iniciando transmissao (%s): %d bytes em %d quadros ---\n",
                (tipoConteudo == CONTEUDO_IMAGEM) ? "IMAGEM" : "TEXTO",
                totalBytes, totalQuadros);

  while (offset < totalBytes) {
    uint8_t tamanhoBloco = min((uint16_t)MAX_PAYLOAD, (uint16_t)(totalBytes - offset));

    Quadro qData;
    qData.type = TIPO_DATA;
    qData.seq  = seq;
    qData.len  = tamanhoBloco;
    memcpy(qData.data, dados + offset, tamanhoBloco);

    bool erroNesteQuadro = (simularErro && seq == 0);

    if (!enviarQuadroConfiavel(qData, erroNesteQuadro)) {
      Serial.println("--- Transmissao abortada por falha de entrega ---\n");
      return;
    }

    offset += tamanhoBloco;
    seq = (seq + 1) % 256;
  }

  Quadro qEnd;
  qEnd.type = TIPO_END;
  qEnd.seq  = seq;
  qEnd.len  = 1;
  qEnd.data[0] = tipoConteudo;

  if (enviarQuadroConfiavel(qEnd, false)) {
    Serial.println("--- Transmissao concluida com sucesso! ---\n");
  } else {
    Serial.println("--- Falha ao confirmar quadro END ---\n");
  }
}

void enviarACK(uint8_t seqConfirmado) {
  delay(20);

  Quadro qAck;
  qAck.type = TIPO_ACK;
  qAck.seq  = seqConfirmado;
  qAck.len  = 0;

  enviarQuadroRF(qAck, false);
  Serial.printf("[TX_ACK] ACK enviado para SEQ=%d\n", seqConfirmado);
}

void processarQuadroRecebido(const Quadro &rx) {
  uint8_t fcsCalculado = calcularFCS(rx);

  if (fcsCalculado != rx.fcs) {
    Serial.printf("[RX_ERRO] Quadro corrompido descartado! FCS rx=0x%02X, esperado=0x%02X\n",
                  rx.fcs, fcsCalculado);
    return;
  }

  if (rx.type == TIPO_DATA) {
    if (!emSessao && rx.seq == 0) {
      emSessao = true;
      seqEsperado = 0;
      ultimoSeqEnd = -1;
      tamanhoRecebido = 0;
    }

    if (emSessao && rx.seq == seqEsperado) {
      if ((tamanhoRecebido + rx.len) <= MAX_BUFFER_DADOS) {
        memcpy(bufferRecepcao + tamanhoRecebido, rx.data, rx.len);
        tamanhoRecebido += rx.len;
      }
      Serial.printf("[RX_DATA] SEQ=%d aceito (%d bytes | acumulado=%d)\n",
                    rx.seq, rx.len, tamanhoRecebido);
      seqEsperado = (seqEsperado + 1) % 256;
      enviarACK(rx.seq);
    } else {
      Serial.printf("[RX_DUP] DATA duplicado SEQ=%d ignorado; reenviando ACK\n", rx.seq);
      enviarACK(rx.seq);
    }
  }
  else if (rx.type == TIPO_END) {
    if (emSessao && rx.seq == seqEsperado) {
      uint8_t tipoConteudo = (rx.len > 0) ? rx.data[0] : CONTEUDO_TEXTO;
      Serial.printf("[RX_END] Fim da transmissao (SEQ=%d | Tipo=0x%02X | Total=%d bytes)\n",
                    rx.seq, tipoConteudo, tamanhoRecebido);

      emSessao = false;
      ultimoSeqEnd = rx.seq;
      seqEsperado = 0;
      enviarACK(rx.seq);

      if (tipoConteudo == CONTEUDO_TEXTO) {
        Serial.println("==========================================");
        Serial.print("MENSAGEM RECEBIDA: ");
        for (uint16_t i = 0; i < tamanhoRecebido; i++) {
          Serial.print((char)bufferRecepcao[i]);
        }
        Serial.println("\n==========================================\n");
        exibirTextoRecebidoLCD(bufferRecepcao, tamanhoRecebido);
      }
      else if (tipoConteudo == CONTEUDO_IMAGEM) {
        exibirNoLCD("Imagem recebida!", String(tamanhoRecebido) + " bytes OK");
        Serial.print("[IMG_HEX]:");
        for (uint16_t i = 0; i < tamanhoRecebido; i++) {
          if (bufferRecepcao[i] < 0x10) Serial.print('0');
          Serial.print(bufferRecepcao[i], HEX);
        }
        Serial.println();
      }
    } else if (!emSessao && rx.seq == ultimoSeqEnd) {
      Serial.printf("[RX_DUP] END duplicado SEQ=%d; reenviando ACK\n", rx.seq);
      enviarACK(rx.seq);
    }
  }
}

uint8_t charParaNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return 0;
}

uint16_t decodificarHexParaBuffer(const String &hexStr, uint8_t* destino, uint16_t maxBytes) {
  uint16_t numBytes = hexStr.length() / 2;
  if (numBytes > maxBytes) numBytes = maxBytes;

  for (uint16_t i = 0; i < numBytes; i++) {
    uint8_t hi = charParaNibble(hexStr[i * 2]);
    uint8_t lo = charParaNibble(hexStr[i * 2 + 1]);
    destino[i] = (hi << 4) | lo;
  }
  return numBytes;
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.begin(115200);

  pinMode(PINO_TX, OUTPUT);
  pinMode(PINO_RX, INPUT);
  digitalWrite(PINO_TX, LOW);

  inicializarLCD();
  ultimoEstadoRX = digitalRead(PINO_RX);

  Serial.println("\n=== Sistema RF 433 MHz - Texto, Imagem, Checksum + ARQ + LCD ===");
  Serial.println("Comandos diretos: <texto> | !erro <texto> | !img <hex> | !img_erro <hex>");
}

void loop() {
  if (Serial.available()) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim();

    if (entrada.length() > 0) {
      if (entrada.startsWith("!img_erro ")) {
        String hexDados = entrada.substring(10);
        uint16_t qtdBytes = decodificarHexParaBuffer(hexDados, bufferTransmissao, MAX_BUFFER_DADOS);
        if (qtdBytes > 0) {
          transmitirBuffer(bufferTransmissao, qtdBytes, CONTEUDO_IMAGEM, true);
        }
      }
      else if (entrada.startsWith("!img ")) {
        String hexDados = entrada.substring(5);
        uint16_t qtdBytes = decodificarHexParaBuffer(hexDados, bufferTransmissao, MAX_BUFFER_DADOS);
        if (qtdBytes > 0) {
          transmitirBuffer(bufferTransmissao, qtdBytes, CONTEUDO_IMAGEM, false);
        }
      }
      else {
        bool simularErro = false;
        if (entrada.startsWith("!erro ")) {
          simularErro = true;
          entrada = entrada.substring(6);
        }

        uint16_t qtdBytes = min((uint16_t)entrada.length(), (uint16_t)MAX_BUFFER_DADOS);
        if (qtdBytes > 0) {
          memcpy(bufferTransmissao, entrada.c_str(), qtdBytes);
          transmitirBuffer(bufferTransmissao, qtdBytes, CONTEUDO_TEXTO, simularErro);
        }
      }
    }
  }

  Quadro rxQuadro;
  if (receberQuadroRF(rxQuadro)) {
    processarQuadroRecebido(rxQuadro);
  }
}