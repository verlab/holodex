import numpy as np



class Cinematics:
    def __init__(self):
        self.motorsDir : list[np.array] = []
        self.motorsPos : list[np.array] = []
    

    def addMotor(self, x_component : float, y_component : float, x_pos : float, y_pos : float):
        vec = np.array([x_component, y_component])
        size1 = np.linalg.norm(vec)
        self.motorsDir.append(vec / size1)

        pos = np.array([x_pos, y_pos])
        size2 = np.linalg.norm(pos)
        self.motorsPos.append(pos / size2)
    
    def getDirection(self, direction_x : float, direction_y : float, rotation : float) -> list[float]:
        l = []
        vec = np.array([direction_x, direction_y])

        # Iteramos pelas direções e posições de cada motor ao mesmo tempo
        for m_dir, m_pos in zip(self.motorsDir, self.motorsPos):
            # Translação: Projeção do vetor de movimento na direção da roda
            translucao = np.dot(vec, m_dir)
            
            # Rotação: Produto vetorial 2D (m_pos x m_dir) multiplicado pela velocidade angular
            # np.cross retorna um escalar que representa (x_pos * y_dir - y_pos * x_dir)
            rotacao = rotation * np.cross(m_pos, m_dir)
            
            # A velocidade final da roda é a soma linear das duas componentes
            l.append(translucao + rotacao)

        return l
