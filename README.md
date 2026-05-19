# 🌱 GrandRoot

**Carrito autónomo para análisis inteligente del suelo.**

GrandRoot es un sistema autónomo terrestre diseñado para **analizar y procesar datos agrícolas en tiempo real**, ayudando a mejorar la toma de decisiones en agricultura de precisión en cuanto a fertilización.

## Tabla de contenidos

- [¿Qué problema resuelve?](#qué-problema-resuelve)
- [Objetivos](#objetivos)
- [Arquitectura del sistema](#arquitectura-del-sistema)
- [Funcionalidades](#funcionalidades)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Tecnologías](#tecnologías)
- [Aplicaciones](#aplicaciones)
- [Estado del proyecto](#estado-del-proyecto)
- [Futuras mejoras](#futuras-mejoras)
- [Equipo](#equipo)
- [Licencia](#licencia)
- [Contacto](#contacto)


## ¿Qué problema resuelve?

En agricultura tradicional suele haber:

- Medición manual del suelo (lenta e imprecisa)
- Falta de monitoreo en tiempo real
- Uso ineficiente de agua y fertilizantes
- Baja integración tecnológica en campo

GrandRoot lo mejora mediante:

- Automatización de fertilización
- Recolección continua de datos
- Procesamiento inteligente
- Soporte para decisiones agrícolas

## Objetivos

- Navegar de forma independiente en terrenos agrícolas.
- Recolectar datos del suelo en tiempo real.
- Procesar información mediante algoritmos inteligentes.
- Generar recomendaciones para el agricultor.
- Aplicar biofertilizantes basado en las necesidades de la planta.

## Arquitectura del sistema

El sistema está compuesto por cinco módulos principales:

1. **🚗 Movilidad (plataforma robótica)**
   - Chasis móvil (ruedas o tracción)
   - Motores y controladores
   - Sistema de dirección
2. **🔍 Sensores**
   - GPS (posicionamiento)
   - IMU (orientación y movimiento)
   - Sensores de proximidad (evitación de obstáculos)
3. **🧠 Unidad de control**
   - Microcontrolador (ESP32)
   - Computadora embebida (Raspberry Pi 5)
   - Procesamiento de datos
4. **📡 Comunicación**
   - WiFi / Bluetooth 
   - Transmisión de datos en tiempo real
   - Integración con la nube 
5. **🤖 Software inteligente**
   - Algoritmos de navegación autónoma
   - Procesamiento de datos agrícolas
   - Análisis predictivo (IA futura)

## Funcionalidades

- Navegación autónoma
- Geolocalización de mediciones
- Evitación de obstáculos
- Monitoreo en tiempo real
- Análisis de condiciones del terreno

## Estructura del repositorio

```text
GrandRoot/
├── Pruebas/
│   ├── esquemas/
│   └── componentes/
├── Control/
│   ├── sensores/
│   └── control_motores/
├── Navegación/
│   ├── navegación/
│   └── procesamiento_datos/
├── Interfaz/
│   ├── marco_teorico/
│   ├── analisis_factibilidad/
│   └── diagramas/
├── Sensores/
│   ├── marco_teorico/
│   ├── analisis_factibilidad/
│   └── diagramas/
├── Comunicación/
│   ├── marco_teorico/
│   ├── analisis_factibilidad/
│   └── diagramas/
├── simulations/
└── README.md
```

## Tecnologías

### Hardware

- Microcontroladores ( ESP32)
- Sensores agrícolas
- Motores DC 
- Módulos de comunicación

### Software

- Python
- C/C++ (ESP32)
- ROS (opcional)
- Algoritmos de navegación por visión

## Aplicaciones

- Agricultura de precisión
- Monitoreo de cultivos
- Optimización de fertilización
- Análisis de fertilidad del suelo
- Investigación agronómica

## Estado del proyecto

🔄 **En desarrollo**

Fases actuales:

- Diseño conceptual ✅
- Desarrollo de hardware 🔄
- Implementación de sensores 🔄
- Programación de navegación ⏳

## Futuras mejoras

- Integración con inteligencia artificial
- App móvil para monitoreo
- Dashboard web
- Uso de drones complementarios
- Análisis predictivo de cultivos

## Equipo

Proyecto desarrollado por estudiantes de ingeniería en:

- **Mecatrónica:**
- Jose Eduardo Mancinas
- Victor Martínez
- Alejandro Ramos
- Enrique Soto
- **Mecánica:**
- José Luis Chavarria
- **Biotecnología:**
- Ana Cristina Leyva
- Melina Valenzuela

## Licencia

Este proyecto se encuentra bajo la licencia **MIT**.

## Contacto

Para colaboraciones o más información:

- Email: A01568916@exatec.tec.mx
- GitHub: EnriqueSotoB
