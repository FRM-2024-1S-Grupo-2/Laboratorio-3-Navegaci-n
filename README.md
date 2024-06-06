# Laboratorio 3 FRM Navegación Robot EV3.

Andres Felipe Zuleta Romero.

Edgar Giovanni Obregon.

Daniel Felipe Cantor Santana.

Thomas Hernandez Ochoa.

# Objetivos.
+ Identificar los distintos algoritmos de navegación para robots móviles.
+ Aplicar al menos dos algoritmos de navegación en el robot EV3.
+ Identificar las caracerísticas que diferencian cada uno de los algoritmos de navegación.

# Resumen.

En esta práctica de laboratorio, se implementaron y compararon dos algoritmos de navegación, BUG y MAZE, en un robot móvil EV3 para llevarlo desde un punto de partida a un punto de destino. El algoritmo BUG permitió al robot seguir los contornos de los obstáculos, mientras que el algoritmo MAZE lo ayudó a resolver laberintos tomando decisiones en las intersecciones. Los resultados mostraron que BUG es más adecuado para entornos con obstáculos dispersos y MAZE para laberintos estructurados, destacando la importancia de elegir el algoritmo correcto según el entorno y los objetivos.

# Introducción.

La navegación autónoma de robots móviles es un área clave en la robótica moderna, con aplicaciones que van desde la exploración espacial hasta la logística en almacenes. En esta práctica de laboratorio, se explora la implementación y comparación de dos algoritmos de navegación, BUG y MAZE, en un robot móvil EV3 de Lego Mindstorms. El objetivo es analizar cómo estos algoritmos permiten al robot alcanzar un destino específico desde un punto de partida, enfrentándose a diferentes configuraciones de obstáculos y laberintos.

El algoritmo BUG se basa en el principio de seguir los contornos de los obstáculos hasta encontrar un camino libre hacia el objetivo, lo que lo hace adecuado para entornos con obstáculos dispersos. Por otro lado, el algoritmo MAZE está diseñado para resolver laberintos, permitiendo al robot tomar decisiones en las intersecciones y encontrar la salida mediante reglas predefinidas. A través de esta práctica, se pretende comprender las fortalezas y limitaciones de cada enfoque, evaluando su rendimiento en diversas situaciones y destacando la importancia de seleccionar el algoritmo de navegación adecuado según el entorno específico. Esta experiencia proporciona una base sólida para futuras investigaciones y aplicaciones en la navegación autónoma de robots.

# Consulta bibliográfica.

A continuación se presenta una breve consulta bibliográfica, la cual es necesaria para comprender y poder desarrollar el laboratorio de navegación en su totalidad.

## Características de la navegación planeada.

+ Ruta predefinida:En la navegación planeada, se traza una ruta específica desde el punto de partida hasta el destino antes de que el robot comience a moverse, en cuanto a la influencia en la respuesta del robot, sigue estrictamente la ruta predefinida, lo que puede resultar en un comportamiento eficiente y predecible en entornos conocidos. Sin embargo, puede tener dificultades para adaptarse a cambios imprevistos en el entorno.
  
+ Mapeo del entorno: Este enfoque requiere un mapa del entorno para planificar la ruta. El robot utiliza sensores para localizar su posición y seguir el camino trazado en el mapa, en cuanto a la influencia en la respuesta del robot, la precisión del movimiento del robot depende de la exactitud del mapa y de su capacidad para posicionarse correctamente. En entornos dinámicos, la navegación planeada puede ser menos efectiva si el mapa no se actualiza continuamente.

## Características de la navegación basada en comportamiento.

+ Reactividad:En la navegación basada en comportamientos, el robot reacciona a los estímulos del entorno en tiempo real utilizando una serie de comportamientos preprogramados, la influencia en la respuesta del robot es que permite al robot adaptarse rápidamente a cambios y obstáculos imprevistos, proporcionando una mayor flexibilidad en entornos dinámicos y no estructurados.
+ Modularidad de comportamientos:El robot utiliza módulos de comportamientos que pueden ser activados en diferentes situaciones, como evitar obstáculos, seguir paredes, o buscar metas, esto influye en que facilita una respuesta más robusta y adaptativa, permitiendo que el robot combine varios comportamientos para navegar eficazmente en situaciones complejas y variadas. Esta adaptabilidad, sin embargo, puede resultar en rutas menos eficientes en términos de distancia recorrida en comparación con la navegación planeada.

## Investigaciones destacadas y robots desarrollados por los robotistas Rodney Brooks y Mark Tilden.

+ Rodney Brooks: Es un influyente robotista conocido por su trabajo en el campo de la robótica basada en comportamientos. Brooks desarrolló la teoría de la inteligencia artificial subsimbólica, que sostiene que el comportamiento inteligente puede surgir de la interacción de agentes simples en lugar de depender de representaciones simbólicas complejas. En 1991, Brooks introdujo el concepto de "robótica de subsunción" a través del diseño del robot Genghis, un robot hexápodo que utilizaba capas de comportamiento para realizar tareas de navegación y exploración. Este enfoque rompió con las metodologías tradicionales de IA y robótica al demostrar que sistemas relativamente simples pueden producir comportamientos complejos y adaptativos.
Además de sus contribuciones teóricas, Brooks cofundó iRobot Corporation, donde ayudó a desarrollar robots comerciales como Roomba, la famosa aspiradora autónoma. Roomba es un ejemplo práctico de su enfoque de robótica basada en comportamientos, utilizando sensores simples y algoritmos de respuesta reactiva para realizar tareas de limpieza doméstica. Su trabajo ha influenciado significativamente tanto la investigación académica como el desarrollo de productos comerciales en robótica, consolidando su posición como un pionero en el campo.

+ Mark Tilden: Es conocido por su trabajo en el desarrollo de la robótica BEAM (Biology, Electronics, Aesthetics, and Mechanics). Tilden se centra en la creación de robots autónomos utilizando principios inspirados en la biología y la física simple, evitando el uso de microprocesadores y software complejo. En lugar de depender de la programación digital, los robots BEAM de Tilden utilizan circuitos analógicos y respuestas reflejas para realizar tareas de navegación y supervivencia. Uno de sus robots más conocidos es el insecto robótico "Solarbot," que utiliza energía solar y circuitos sencillos para moverse y reaccionar a su entorno.
Tilden también es conocido por su trabajo en la industria del entretenimiento y juguetes robóticos, especialmente durante su tiempo en WowWee, donde desarrolló el robot juguete Robosapien. Robosapien es un humanoide interactivo que combina características de robótica BEAM con elementos de control digital, permitiendo una variedad de movimientos y comportamientos preprogramados. El éxito comercial de Robosapien y otros productos de Tilden ha demostrado la viabilidad de enfoques alternativos en robótica, destacando la eficacia de diseños simples y robustos para aplicaciones tanto lúdicas como funcionales.

## Tres algoritmos de planeación de rutas para espacios con obstáculos.

+ Algoritmo de Dijkstra: Este es uno de los algoritmos más antiguos y simples para la planificación de rutas. Partiendo de un vértice inicial, el algoritmo marca todos los vecinos directos con el costo para llegar allí y luego procede al vértice con el menor costo, repitiendo el proceso hasta alcanzar el vértice objetivo.
+ Algoritmo A*: Es una mejora del algoritmo de Dijkstra que utiliza heurísticas para guiar la búsqueda. A* es eficiente y suele encontrar el camino más corto más rápidamente que Dijkstra, especialmente en espacios con muchos obstáculos.
+ Planificación Basada en Diagramas de Voronoi: Este método utiliza los diagramas de Voronoi para la navegación y evasión de obstáculos. Los diagramas de Voronoi pueden ayudar a crear un camino que maximiza la distancia mínima a los obstáculos, lo que es útil para la planificación de trayectorias en entornos desconocidos o dinámicos.

## Descripción de los algoritmos Bug 0, Bug 1 y Bug 2.

+ BUG 0: Este es un algoritmo reactivo simple para la navegación de robots en entornos con obstáculos. El robot se mueve en línea recta hacia el objetivo hasta que encuentra un obstáculo. Luego sigue el contorno del obstáculo manteniendo contacto con él hasta que puede retomar su camino en línea recta hacia el objetivo.
+ BUG 1: Similar al BUG 0, el robot se dirige directamente hacia el objetivo hasta que topa con un obstáculo. La diferencia es que el robot rodea completamente el obstáculo y registra el punto más cercano al objetivo. Después de rodear el obstáculo, el robot regresa a ese punto y continúa su trayectoria hacia el objetivo.
+ BUG 2: Este algoritmo mejora el BUG 1 al evitar la necesidad de rodear completamente el obstáculo. Cuando el robot encuentra un obstáculo, sigue su contorno solo hasta que su trayectoria hacia el objetivo está despejada, y luego continúa directamente hacia el objetivo.

## Algoritmo de solución de un laberinto empleado en robótica móvil.

El “maze algorithm” o algoritmo de laberinto es un conjunto de técnicas utilizadas en robótica móvil para permitir que un robot autónomo encuentre su camino a través de un laberinto o entorno con obstáculos. Aquí hay una descripción general de cómo se aplica este algoritmo en robótica móvil:

+ Mapeo: El robot utiliza sensores para identificar las paredes y pasillos del laberinto, creando un mapa del entorno.
+ Planificación de la ruta: Con el mapa creado, el robot planifica una ruta hacia la salida. Esto puede implicar algoritmos como A* o Dijkstra, que ayudan a encontrar la ruta más corta y eficiente.
+ Navegación: El robot sigue la ruta planificada, ajustándose según sea necesario si encuentra obstáculos no mapeados o cambios en el entorno.
+ Resolución de desafíos: Si el robot llega a un punto muerto o necesita reevaluar su ruta, puede utilizar estrategias como el algoritmo de la mano derecha, que implica seguir siempre la pared derecha hasta encontrar la salida.

# Mision 1
# Proyecto de Navegación con Algoritmo BUG2

## Descripción del Proyecto

Este proyecto tiene como objetivo implementar el algoritmo BUG2 para navegar desde una posición inicial P1 hasta una posición final P2 sorteando dos bostaculos como muestra la imagen. La misión 1 se enfoca en la navegación utilizando el algoritmo BUG2 para evitar obstáculos y alcanzar el destino deseado.

![image](https://github.com/FRM-2024-1S-Grupo-2/Laboratorio-3-Navegacion/assets/82681128/27c29c6c-6a8e-4176-9ab0-f4dddd3e14d3)

### Descripción de la Solución Seleccionada

Para la misión 1, hemos seleccionado el algoritmo BUG2 debido a su capacidad para manejar la navegación en entornos con obstáculos. El algoritmo BUG2 sigue una estrategia donde el robot se mueve hacia el objetivo hasta que encuentra un obstáculo. En ese momento, el robot sigue el contorno del obstáculo hasta encontrar un punto donde pueda reanudar su movimiento directo hacia el objetivo.

### Algoritmo BUG2 en Pseudocódigo

```pseudo
Inicio:
    Inicializar posición P1 y posición objetivo P2
    Mientras la posición actual != P2:
        Si hay una línea de vista directa hacia P2:
            Moverse en línea recta hacia P2
        Sino:
            Seguir el contorno del obstáculo hasta encontrar un punto que permita reanudar la línea recta hacia P2
Fin
```

### Vídeo de la Implementación

Se adjunta un vídeo demostrando la implementación del algoritmo BUG2 para la misión 1. El vídeo muestra al robot navegando desde P1 hasta P2, superando obstáculos en su camino.
[!Ver video](https://youtu.be/dtWheftjFDU)

[!Ver video](https://youtu.be/NkHWqsUWm2k)


# Mision 2
# Proyecto de Navegación en Laberinto con Algoritmo MAZE

## Descripción del Proyecto

Esta mision tiene como objetivo implementar un algoritmo de resolución de laberintos para navegar desde una entrada P1 hasta una salida P2. La misión 2 se enfoca en la navegación utilizando un algoritmo específico de MAZE que revisa ambos lados cuando detecta un obstáculo al frente y gira hacia el lado que tenga más espacio disponible.

![image](https://github.com/FRM-2024-1S-Grupo-2/Laboratorio-3-Navegacion/assets/82681128/122662b5-a05c-4ff2-82c5-1d6d1dbe3067)

### Descripción de la Solución Seleccionada

Para la misión 2, hemos seleccionado un algoritmo que detecta obstáculos y elige la dirección con más espacio disponible para continuar la navegación. Este enfoque es efectivo para encontrar caminos libres y evitar quedarse atrapado en callejones sin salida.

### Algoritmo MAZE en Pseudocódigo

```pseudo
Inicio:
    Inicializar posición P1 y posición objetivo P2
    Mientras la posición actual != P2:
        Si no hay obstáculo al frente:
            Moverse hacia adelante
        Sino:
            Revisar espacio a la izquierda y a la derecha
            Si el espacio a la izquierda > espacio a la derecha:
                Girar a la izquierda
            Sino:
                Girar a la derecha
Fin
```

### Vídeo de la Implementación

Se adjuntan dos vídeos demostrando la implementación de la mision 2. El primer vídeo muestra al robot navegando desde P1 hasta P2, superando el laberinto. Mientras que la idea  del segundo video es que pueda verificarse como va actuando el código:

[!Ver video](https://youtu.be/c8UgCnnJz58?si=xPmGtbyE9bXLynKj)
[!Ver video](https://youtu.be/l_cYp5f-If8?si=S12II0bMkwv5WBVw)



