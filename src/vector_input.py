import sys
import math
from PySide6.QtCore import Qt, Signal, QPointF, Slot
from PySide6.QtGui import QPainter, QPen, QBrush, QColor
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton

class Vetor2DWidget(QWidget):
    # Sinal emitido sempre que o vetor mudar (retorna X e Y)
    vector = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(250, 250)
        self.vx = 0.0
        self.vy = 0.0
        self.arrastando = False

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Determina o centro e o raio máximo do círculo funcional
        largura, altura = self.width(), self.height()
        cx, cy = largura / 2, altura / 2
        raio = min(cx, cy) - 15  # 15px de margem das bordas

        # 1. Desenha o fundo do círculo unitário
        painter.setPen(QPen(QColor("#bdc3c7"), 2))
        painter.setBrush(QBrush(QColor("#333")))
        painter.drawEllipse(QPointF(cx, cy), raio, raio)

        # 2. Desenha os eixos cartesianos discretos (X e Y)
        painter.setPen(QPen(QColor("#e2e8f0"), 1, Qt.PenStyle.DashLine))
        painter.drawLine(cx - raio, cy, cx + raio, cy)
        painter.drawLine(cx, cy - raio, cx, cy + raio)

        # 3. Calcula a posição do ponto na tela com base no vetor (vx, vy)
        # Invertemos o Y porque na tela o Y cresce para baixo
        px = cx + self.vx * raio
        py = cy - self.vy * raio

        # 4. Desenha a linha do vetor (do centro até o ponto)
        painter.setPen(QPen(QColor("#3498db"), 3))
        painter.drawLine(QPointF(cx, cy), QPointF(px, py))

        # 5. Desenha a "manopla"/indicador do vetor
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor("#e74c3c")))
        painter.drawEllipse(QPointF(px, py), 8, 8)

    def atualizar_vetor_por_posicao(self, posicao_mouse):
        cx, cy = self.width() / 2, self.height() / 2
        raio = min(cx, cy) - 15

        if raio <= 0:
            return

        # Calcula a distância relativa ao centro do widget
        dx = posicao_mouse.x() - cx
        dy = cy - posicao_mouse.y()  # Inverte para o padrão matemático

        # Converte a posição de pixels para a escala -1 a 1
        x = dx / raio
        y = dy / raio

        # Teorema de Pitágoras para calcular o comprimento atual
        comprimento = math.sqrt(x*x + y*y)

        # SE passar de 1, limitamos (normalizamos) o vetor na borda do círculo
        if comprimento > 1.0:
            x /= comprimento
            y /= comprimento

        self.vx = x
        self.vy = y
        
        # Emite o sinal com os novos valores e força o widget a se redesenhar
        self.vector.emit(self.vx, self.vy)
        self.update()
        
    def set_value(self, vx, vy):
        self.vx = vx
        self.vy = vy
        
        self.vector.emit(self.vx, self.vy)
        self.update()
        

    # Eventos de mouse para permitir o arrasto intuitivo
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.arrastando = True
            self.atualizar_vetor_por_posicao(event.position())

    def mouseMoveEvent(self, event):
        if self.arrastando:
            self.atualizar_vetor_por_posicao(event.position())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.arrastando = False


# --- Widget testing window ---
class JanelaPrincipal(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Seletor de Vetor 2D")
        
        layout = QVBoxLayout(self)
        
        # Cria o nosso widget customizado
        self.seletor_vetor = Vetor2DWidget()
        layout.addWidget(self.seletor_vetor)
        
        # button
        self.button = QPushButton("Reset")
        layout.addWidget(self.button)
        self.button.clicked.connect(self.reset_button)
        
        # Labels para mostrar o resultado em tempo real
        self.label_status = QLabel("Arraste o ponto vermelho")
        self.label_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #2c3e50;")
        layout.addWidget(self.label_status)
        
        # Conecta o sinal do widget para atualizar os textos na tela
        self.seletor_vetor.vector.connect(self.exibir_valores)
        
        
    @Slot() 
    def reset_button(self):
        self.seletor_vetor.set_value(0,0)
        print("testando")
        
    def exibir_valores(self, x, y):
        comprimento = math.sqrt(x*x + y*y)
        texto = f"X: {x:.3f}  |  Y: {y:.3f}\nComprimento: {comprimento:.3f}"
        self.label_status.setText(texto)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    janela = JanelaPrincipal()
    janela.show()
    sys.exit(app.exec())