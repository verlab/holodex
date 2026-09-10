# Guia de Reprodução: Quantização Estática INT8 e Geração do Binário NBG (.nb)

Este documento descreve detalhadamente o passo a passo para reproduzir a conversão, poda de nós incompatíveis, calibração estática INT8 e compilação do modelo de segmentação de instâncias **YOLOv8-seg** para a NPU **VeriSilicon Vivante VIP9000** (SoC Allwinner A733, Target ID `0x1000003b`).

---

## 1. Visão Geral do Pipeline e Hardware Alvo

* **Hardware Alvo**: Orange Pi baseada no SoC Allwinner A733
* **NPU**: VeriSilicon Vivante VIP9000 (Target ID: `0x1000003b` / Device `VIP9000`)
* **Runtime de Inferência na Placa**: VIPLite (`vip_lite.h`, `libVIPhal.so`)
* **Ferramentas de Build no Host / Container**:
  * **Acuity Toolkit**: v6.30.22 (`pegasus`)
  * **Vivante IDE / Cmdtools**: v5.11.0 (`cmdtools/vsimulator`)
* **Estratégia de Execução**:
  * **NPU**: Executa todas as convoluções e extrações de features em INT8 puro (incluindo o gerador de protótipos de máscara Proto).
  * **CPU**: Executa o pós-processamento em C++ (decodificação DFL das caixas, sigmoid de classes, NMS, multiplicação matricial Proto x coeficientes e recorte de máscara).

---

## 2. Poda do Modelo ONNX (`prune.py`)

Modelos YOLO padrão exportados via Ultralytics contêm operações dinâmicas pós-convolução (DFL, Reshapes dinâmicos, Permute e Concatenações com transposição de eixos) que não são suportadas ou são altamente ineficientes em NPUs embarcadas.

O script a seguir extrai um grafo limpo contendo estritamente **10 saídas convolucionais estáticas 4D**:
1. O tensor de protótipos de máscara **Proto** (`output0`: `[1, 32, 104, 104]`).
2. As **9 saídas convolucionais terminais** da cabeça (`model.22`), correspondentes às 3 escalas (P3, P4, P5) e 3 ramos (Box `cv2`, Class `cv3`, Mask Coeficients `cv4`).

Crie ou utilize o arquivo `/workspace/prune.py`:

```python
import onnx
from onnx import utils

# 1. Carrega o modelo ONNX original (opset 12)
model = onnx.load("yolov8n-seg.onnx")

# 2. O tensor de protótipos de máscara Proto é a saída limpa original (output0: [1, 32, 104, 104])
proto_output = model.graph.output[0].name
print(f"Tensor de protótipos (Proto): {proto_output}")

# 3. Filtra apenas as 9 convoluções finais da Head (model.22)
# 3 escalas (P3=stride 8, P4=stride 16, P5=stride 32) x 3 branches (cv2, cv3, cv4)
head_outputs = []
for node in model.graph.node:
    if "model.22" in node.name and node.op_type == "Conv":
        if any(k in node.name for k in [
            "cv2.0.2", "cv2.1.2", "cv2.2.2",  # Boxes (64 canais = 4 x 16 bins DFL)
            "cv3.0.2", "cv3.1.2", "cv3.2.2",  # Classes (80 classes COCO)
            "cv4.0.2", "cv4.1.2", "cv4.2.2",  # Coeficientes de máscara (32 canais)
        ]):
            head_outputs.append(node.output[0])
            print(f"Terminal encontrada: {node.name} -> {node.output[0]}")

target_outputs = [proto_output] + head_outputs
print(f"\nTotal de tensores de saída alvo: {len(target_outputs)} (Esperado: 10)")

# 4. Extrai o subgrafo limpo
if len(target_outputs) == 10:
    utils.extract_model(
        input_path="yolov8n-seg.onnx",
        output_path="yolov8n_pruned.onnx",
        input_names=["images"],
        output_names=target_outputs,
    )
    print("\nGrafo podado com sucesso em 'yolov8n_pruned.onnx'!")
else:
    raise RuntimeError(f"Esperado 10 saídas, encontrado {len(target_outputs)}")
```

Execute a poda:
```bash
python3 /workspace/prune.py
```

---

## 3. Importação para o Formato Intermediário Acuity

> **Atenção à sintaxe do `--input-size-list`:** O Acuity assume formato sem o batch por padrão. Passar `"1,3,416,416"` criará um tensor 5D que quebra o inferenciador espacial da convolução. Deve-se passar `"3,416,416"`, além do nome explícito da entrada `--inputs "images"`.

Execute o comando de importação:
```bash
python3 /root/acuity-toolkit-whl-6.30.22/bin/pegasus.py import onnx \
  --model /workspace/yolov8n_pruned.onnx \
  --output-data /workspace/yolov8n_pruned.data \
  --output-model /workspace/yolov8n_pruned.json \
  --inputs "images" \
  --input-size-list "3,416,416"
```

---

## 4. Configuração dos Metadados de Entrada (`inputmeta.yml`)

Gere o template de metadados de entrada:
```bash
python3 /root/acuity-toolkit-whl-6.30.22/bin/pegasus.py generate inputmeta \
  --model /workspace/yolov8n_pruned.json \
  --input-meta-output /workspace/yolov8n_pruned_inputmeta.yml
```

Edite o arquivo `/workspace/yolov8n_pruned_inputmeta.yml` para configurar a normalização padrão do YOLO (1/255.0 = 0.00392156862745098) e a inversão de canais BGR -> RGB:

```yaml
input_meta:
  databases:
  - path: dataset.txt
    type: TEXT
    ports:
    - lid: images_250
      category: image
      dtype: float32
      sparse: false
      tensor_name:
      layout: nchw
      shape:
      - 1
      - 3
      - 416
      - 416
      fitting: scale
      preprocess:
        reverse_channel: true
        mean:
        - 0
        - 0
        - 0
        scale:
        - 0.00392156862745098
        - 0.00392156862745098
        - 0.00392156862745098
        preproc_node_params:
          add_preproc_node: false
          preproc_type: IMAGE_RGB
          preproc_image_size:
          - 416
          - 416
          preproc_crop:
            enable_preproc_crop: false
            crop_rect:
            - 0
            - 0
            - 416
            - 416
          preproc_perm:
          - 0
          - 1
          - 2
          - 3
      redirect_to_output: false
```

> **Nota:** O arquivo `/workspace/dataset.txt` deve conter os caminhos absolutos para imagens representativas do dataset de calibração (por exemplo, 50 imagens do COCO).

---

## 5. Quantização Estática INT8 (`pegasus quantize`)

> **Uso obrigatório de `--rebuild-all`**: Sem essa flag, o quantizador pode não calibrar nós intermediários após fusões de ativação Swish, gerando erros de nós mistos (`ASYM UINT8, FLOAT16`) durante a compilação do NBG. A flag `--rebuild-all` força a quantização de todos os 356 tensores do grafo.

Execute a quantização:
```bash
python3 /root/acuity-toolkit-whl-6.30.22/bin/pegasus.py quantize \
  --model /workspace/yolov8n_pruned.json \
  --model-data /workspace/yolov8n_pruned.data \
  --model-quantize /workspace/yolov8n_pruned.quantize \
  --with-input-meta /workspace/yolov8n_pruned_inputmeta.yml \
  --device CPU \
  --quantizer asymmetric_affine \
  --qtype int8 \
  --rebuild-all \
  --batch-size 1 \
  --iterations 50
```

---

## 6. Configuração de Otimização da NPU VIP9000 (`vip9000.yml`)

Crie o arquivo `/workspace/vip9000.yml` que referencia a configuração de hardware do SoC Allwinner A733 encontrada no SDK:

```yaml
target: VIP9000NANODI_PID0X1000003B
pid: "0x1000003B"
soc:
  axi_sram_size: "0x0"
  ext_sram_sizes: "0x0"
```

> **Nota:** Os valores hexadecimais como `"0x1000003B"` e `"0x0"` **devem estar entre aspas** para evitar que o parser YAML do Acuity os trate como inteiros e dispare erro de tipo em `int(val, 16)`.

---

## 7. Exportação do Código C OpenVX (`export ovxlib`)

Antes de rodar a exportação, garanta que o `LD_LIBRARY_PATH` esteja limpo (sem bibliotecas do simulador) para evitar conflitos de símbolos com o PyTorch no Python:

```bash
LD_LIBRARY_PATH="" python3 /root/acuity-toolkit-whl-6.30.22/bin/pegasus.py export ovxlib \
  --model /workspace/yolov8n_pruned.json \
  --model-data /workspace/yolov8n_pruned.data \
  --model-quantize /workspace/yolov8n_pruned.quantize \
  --with-input-meta /workspace/yolov8n_pruned_inputmeta.yml \
  --dtype quantized \
  --optimize /workspace/vip9000.yml \
  --output-path /workspace/export_ovxlib
```

---

## 8. Ajustes no Makefile e Compilação do Executável NBG

No arquivo `/workspace/makefile.linux`, adicione o RPATH e as bibliotecas de simulação necessárias (`-lArchModelSw -lNNArchPerf`):

1. Linhas 14 e 17 (LFLAGS):
   ```makefile
   LFLAGS+=-g -Wl,-rpath,$(VIVANTE_SDK_DIR)/lib -Wl,-rpath,$(VIVANTE_SDK_DIR)/../common/lib
   ...
   LFLAGS+=-O3 -Wl,-rpath,$(VIVANTE_SDK_DIR)/lib -Wl,-rpath,$(VIVANTE_SDK_DIR)/../common/lib
   ```
2. Linha 20 (LIBS):
   ```makefile
   LIBS+= -L$(VIVANTE_SDK_DIR)/lib \
    -lOpenVX -lOpenVXU -lCLC -lVSC -lGAL -lovxlib -lEmulator -lvdtproxy -lArchModelSw -lNNArchPerf
   ```

Compile o gerador de binário:
```bash
USE_IDE_LIB=1 VIVANTE_SDK_DIR=/root/Vivante_IDE/VivanteIDE5.11.0/cmdtools/vsimulator make -f makefile.linux clean
USE_IDE_LIB=1 VIVANTE_SDK_DIR=/root/Vivante_IDE/VivanteIDE5.11.0/cmdtools/vsimulator make -f makefile.linux
```
Isso produzirá o executável `/workspace/exportovxlib`.

---

## 9. Compilação do Binário NBG Final (`.nb`)

Configure as variáveis de ambiente essenciais para a execução do simulador:

```bash
# 1. SDK e Arquivo de Configuração do Target Allwinner A733
export VIV_SDK=/root/Vivante_IDE/VivanteIDE5.11.0/cmdtools
export VSIMULATOR_CONFIG=VIP9000NANODI_PID0X1000003B
export VIVANTE_SDK_DIR=/root/Vivante_IDE/VivanteIDE5.11.0/cmdtools/vsimulator
export LD_LIBRARY_PATH=/root/Vivante_IDE/VivanteIDE5.11.0/cmdtools/vsimulator/lib:/root/Vivante_IDE/VivanteIDE5.11.0/cmdtools/common/lib

# 2. Prevenção do OOM Killer (limita a pré-alocação mmap de 4GB para 256MB)
export VSIMULATOR_CONTIGUOUS_SIZE=268435456

# 3. Habilitação do salvamento do binário NBG
export VIV_VX_ENABLE_SAVE_NETWORK_BINARY=1

# 4. CRÍTICO: Deve apontar para o NOME COMPLETO DO ARQUIVO com extensão .nb (não diretório!)
export VIV_VX_SAVE_NETWORK_BINARY_PATH=/workspace/yolov8n_seg_final.nb

# 5. Execução do compilador
/workspace/exportovxlib /workspace/export_ovxlib.export.data /workspace/images_241_0.tensor
```

Saída esperada:
```text
Create Neural Network: 107ms or 107824us
Verify...
Verify Graph: 7175ms or 7175485us
Start run graph [1] times...
Run the 1 time: 81.06ms or 81064.95us
vxProcessGraph execution time:
Total   81.14ms or 81138.55us
```

---

## 10. Validação do Binário Gerado (`nbinfo`)

Inspecione o binário com a ferramenta `/root/nbinfo`:

```bash
/root/nbinfo -b /workspace/yolov8n_seg_final.nb
```

Confirme os parâmetros:
* **Target**: `0x1000003b`
* **Version**: `0x20000` (NBG v2 compatível com VIPLite)
* **Input Count**: `1` (`[416, 416, 3, 1]` em INT8)
* **Output Count**: `10` (1 tensor Proto de protótipos de máscara `[104, 104, 32, 1]` + 9 tensores terminais das cabeças de detecção/segmentação).
