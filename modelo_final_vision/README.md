# Modelo Final de Visión - Detección de Surcos

## Descripción
Modelo U-Net entrenado con fotos capturadas en el campo de prueba.
Utiliza **pseudo-labeling** para adaptarse al entorno específico.

## Especificaciones
- **Arquitectura**: U-Net (Segmentación Semántica)
- **Tamaño de entrada**: 128x128 píxeles
- **Fotos utilizadas**: 127
- **Estrategia**: Fine-tuning con learning rate muy bajo (1e-5)
- **Epochs**: 40

## Métricas Finales
- **Pérdida (Val)**: 0.5392
- **Dice (Val)**: 0.5411
- **Métrica IoU**: Intersección sobre Unión

## Archivos
- `modelo_final.h5`: Modelo entrenado (usar con custom_objects)
- `ficha_tecnica.json`: Detalles del entrenamiento
- `historial_entrenamiento.png`: Gráficos de pérdida y métricas

## Uso
```python
import tensorflow as tf
from vision_computacional.test_model import CUSTOM_OBJECTS

model = tf.keras.models.load_model(
    'modelo_final_vision/modelo_final.h5',
    custom_objects=CUSTOM_OBJECTS
)
```

## Notas
- Este modelo está optimizado para el ángulo de captura específico de tu cámara
- La generación de pseudo-etiquetas se hizo automáticamente sin anotación manual
- El learning rate bajo (1e-5) preserva el conocimiento del modelo pre-entrenado
