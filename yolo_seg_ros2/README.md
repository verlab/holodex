# YOLOv8-seg NPU Inference Node (ROS 2 Humble / Allwinner A733)

Nó C++ de inferência e segmentação de instâncias para o modelo **YOLOv8n-seg** acelerado por hardware na **NPU VeriSilicon VIP9000 (3 TOPS)** da **Orange Pi 4 Pro** (SoC Allwinner A733), integrado ao ecossistema **ROS 2 Humble** e à câmera **Orbbec Astra Pro**.

---

## 1. Arquitetura

1. **Entrada de Vídeo**: Subscrição no tópico `/camera/color/image_raw` (`sensor_msgs::msg::Image`, `rgb8`).
2. **Pré-processamento (CPU ARM)**:
   - Letterbox simétrico mantendo proporção (416x416).
   - Conversão de cores e quantização direta para INT8 planar (`q = pixel - 128`).
3. **Inferência Neural (NPU VIP9000 / VIPLite)**:
   - Execução pura em hardware a 1008 MHz via `vip_run_network()`.
   - Tempo de inferência na NPU: **~4.8 ms**.
4. **Pós-processamento (CPU ARM)**:
   - Decodificação DFL (Distribution Focal Loss) das bounding boxes nas 3 escalas (P3, P4, P5).
   - Sigmoide e thresholding das 80 classes COCO.
   - Supressão de Não-Máximos (NMS).
   - Multiplicação matricial (Matmul) dos 32 coeficientes com os protótipos de máscara (104x104x32).
   - Recorte ROI da máscara e interpolação de volta para as dimensões originais da câmera (640x480).
5. **Saída**:
   - Geração de imagem anotada (`resultado_yolov8_seg.jpg`) com caixas delimitadoras, rótulos COCO, confiança (%) e máscaras translúcidas coloridas.

---

## 2. Como Compilar na Orange Pi

```bash
cd ~/yolo_seg_ros2
mkdir -p build && cd build
source /opt/ros/humble/setup.bash
cmake ..
make -j4
```

---

## 3. Como Executar

### Opção A: Execução Automatizada com Script (Recomendado)
O script detecta se a câmera Astra Pro já está rodando. Se não estiver, ele a inicia em segundo plano, aguarda a publicação dos frames, roda a inferência de 1 frame e salva o resultado:

```bash
cd ~/yolo_seg_ros2
./scripts/run_yolov8_inference.sh
```

Opções suportadas pelo script:
- `--keep-camera`: Mantém a câmera rodando após a captura do frame.
- `--model <caminho.nb>`: Especifica outro arquivo `.nb`.
- `--output <saida.jpg>`: Caminho para o arquivo gerado.
- `--topic <topico>`: Tópico alternativo de imagem.

### Opção B: Execução Manual do Binário ROS 2
Com a câmera já em execução:
```bash
source /opt/ros/humble/setup.bash
~/yolo_seg_ros2/build/yolo_seg_node --model /home/orangepi/yolov8n_seg_final.nb --output resultado.jpg
```

### Opção C: Teste com Imagem Estática (Sem Câmera)
```bash
~/yolo_seg_ros2/build/yolo_seg_node --model /home/orangepi/yolov8n_seg_final.nb --image /opt/yolov5/input_data/dog.jpg --output resultado_dog.jpg
```

---

## 4. Integração com a Odometria 3D (pointcloud_odometry)

O nó `pointcloud_filter_component` do pacote [`pointcloud_odometry`](../pointcloud_odometry) já possui a arquitetura pronta para assinar a máscara de segmentação de instâncias dinâmicas:
- **Tópico de Máscara**: `/yolo/dynamic_mask` (`sensor_msgs/msg/Image`, encoding `mono8`).
- **Comportamento Atual**: O filtro opera com `enable_dynamic_filter: false` por padrão (economizando CPU/NPU enquanto o modelo neural está sendo calibrado).
- **Ativação Futura**: Ao definir `enable_dynamic_filter: true`, o filtro descarta em tempo real qualquer ponto 3D cujas coordenadas $(u, v)$ correspondam a pixels dinâmicos (pessoas, animais, bolas, etc.), impedindo que a odometria 3D sofra corrupção por movimento dinâmico no ambiente.

