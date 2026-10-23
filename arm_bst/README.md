# Proyecto: Brazo robótico clasificador de cubos por peso (arm_bst)

Este proyecto simula en **Gazebo Harmonic** un brazo robótico de 4 grados de libertad con un gripper tipo "jaula/pala" y un **sensor de fuerza/torque (F/T)** en la muñeca. El brazo toma cada uno de 5 cubos (de 60 g a 450 g) colocados sobre pedestales, los pesa con el sensor y guarda cada resultado (peso, posición angular y radio) en un **Árbol Binario de Búsqueda (BST)**. Un menú en consola permite consultar el árbol (por peso, mínimo, máximo, rango, nivel, ancestros, recorridos), ordenar las lecturas con **Merge Sort**, buscarlas con **Búsqueda Binaria** y ordenar al brazo que levante el cubo elegido. El catálogo de cubos y los pesos medidos se **leen y escriben en archivos CSV**.

Estructura del proyecto:

```
arm_bst/
├── CMakeLists.txt
├── README.md
├── control_bst.cc          programa de control, escaneo, menú y consultas
├── bst.h                   plantilla BST<T> (árbol binario de búsqueda)
├── sorts.h                 plantilla Sorts<T> (Merge Sort y Búsqueda Binaria, del avance 1)
├── cubo.h                  struct Cubo / InfoCubo y lectura/escritura de archivos CSV
├── datos/
│   └── cubos.csv           catálogo de cubos (entrada)  [pesos.csv se genera al escanear]
├── worlds/
│   └── brazo_cubos.sdf     mundo de Gazebo (brazo, sensor F/T, pedestales y cubos)
└── pruebas/
    └── pruebas_datos.cc    pruebas sin Gazebo de archivos, sorts y BST
```

## Descripción del avance 1

En el primer avance se implementó la clase de plantilla `Sorts<T>` dentro de `sorts.h`, con algoritmos de ordenamiento y búsqueda. En `main.cpp` se generaba un vector de estado (`vector<float>`) con lecturas simuladas de peso (en gramos) de los sensores de un robot clasificador. Los datos se ordenaban con **Merge Sort** (`ordenaMerge`) y después se verificaba si existía una carga específica con **Búsqueda Binaria** (`busqBinaria`).

## Descripción del avance 2

En este avance el procesamiento de datos se integra a una simulación física completa. El vector de pesos simulados se sustituye por pesos **medidos** por el sensor F/T de la muñeca del brazo y se guardan en un **BST** de objetos `Cubo` (`id`, `peso`, `q1`, `r`) ordenado por peso. La clase `Sorts<T>` del primer avance se conserva en `sorts.h` y se usa sobre un `vector<Cubo>` con las lecturas.

Flujo del programa:

1. **Lectura del catálogo.** Al iniciar se lee `datos/cubos.csv` (color, posición y peso nominal de cada cubo) con `cargarCatalogo`. Si el archivo no existe o tiene errores, el programa termina con un mensaje que indica la línea del problema, antes de mover el brazo.
2. **Pesos guardados (opcional).** Si existe `datos/pesos.csv`, el programa ofrece cargarlo con `cargarPesos` y reconstruir el BST sin escanear (el escaneo físico tarda aproximadamente 1.5 minutos).
3. **Escaneo (`escanear`)**: para cada cubo el brazo viaja a su vertical (cinemática inversa), mide la *tara* con la mano vacía, baja, cierra el gripper alrededor del cubo y lo levanta. Con la mano cargada mide la fuerza, calcula `masa = |F_con_cubo − F_tara| / g`, devuelve el cubo a su pedestal, lo inserta en el BST (`arbol.add`) y lo agrega al vector de lecturas. Al terminar guarda los pesos en `datos/pesos.csv`.
4. **Menú de consultas** sobre el BST (listado ordenado, vista del árbol, búsqueda por ID o por peso, mínimo, máximo, rango, nivel y ancestros), **Merge Sort + Búsqueda Binaria** (opción 10) y guardado/carga de pesos (opciones 11 y 12).
5. **Acción física**: las opciones 4 a 7 hacen que el brazo levante el cubo encontrado.

## Cambios sobre el primer avance

1. **De vector + Merge Sort a un BST como estructura principal.** En el primer avance el lote de datos se ordenaba una sola vez y quedaba estático. Ahora los datos se miden uno por uno y el menú permite re-escanear (opción 1). Un BST mantiene los datos ordenados *mientras se insertan* (inserción O(h)), mientras que con un vector habría que reordenar todo o insertar en medio con costo O(n). El recorrido inorden del BST entrega los cubos de menor a mayor peso.
2. **`Sorts<T>` se conserva y se usa sobre el vector de lecturas (opción 10).** Merge Sort ordena una copia del vector de lecturas y se compara contra el recorrido inorden del BST (deben coincidir); la Búsqueda Binaria localiza un peso en el vector ordenado y se compara con `find` del BST. Así ambos mecanismos se verifican entre sí. Para esto se añadió `operator<=` a `Cubo` (lo exige `mergeArray`) y en `sorts.h` se cambió `using namespace std;` por `std::` explícito para no contaminar el resto del programa; los algoritmos no se modificaron.
3. **El dato pasa de `float` a la estructura `Cubo`.** Ahora hay que conservar, junto con el peso, la posición del cubo (`q1`, `r`) para que el brazo pueda ir por él. Se sobrecargan `operator<` (orden por peso), `operator==` (igualdad con tolerancia de 5 g, porque el peso medido nunca es exacto) y `operator<<` (impresión). La definición vive ahora en `cubo.h`.
4. **Los datos dejan de estar escritos en el código.** Antes la tabla de cubos (color, `q1`, `r`, peso nominal) estaba fija en `control_bst.cc`. Ahora se lee de `datos/cubos.csv` con validación, de modo que cambiar un cubo no requiere recompilar. Los pesos medidos también se guardan y se pueden volver a cargar, lo que evita repetir el escaneo en cada ejecución.
5. **Los pesos provienen del sensor.** Los pesos del BST son los medidos en el tópico `/wrist_ft` del sensor F/T. Se mide primero una tara con la mano vacía y se resta, para cancelar el peso del propio gripper y cualquier offset constante del sensor.
6. **Agarre por jaula en lugar de soldar el cubo.** El mundo ya no usa `DetachableJoint` (soldaban los cubos a la mano al arrancar). El gripper encapsula el cubo sin apretarlo (labios inferiores, placas laterales con 1 a 2 mm de holgura y paredes frontal y trasera), de modo que el sensor mide el peso real cargado por la mano. La constante `USAR_ATTACH` quedó en `false`.
7. **Se agregan métodos al árbol original** (`find`, `findIf`, `minimum`, `maximum`, `range`, `inorderApply`, `printTree`, `size`, `clear`) porque el menú del brazo los necesita. Se prohíbe copiar el árbol (constructor de copia y asignación privados) porque administra memoria dinámica con punteros.
8. **Corrección del archivo de mundo.** `brazo_cubos.sdf` tenía una etiqueta `<plugin>` abierta y sin cerrar antes del controlador de `joint4`, lo que dejaba el XML mal formado y Gazebo no podía cargarlo. Se eliminó la apertura sobrante.

## Instrucciones para compilar el avance de proyecto

Requisitos: compilador con C++17 y CMake. Para el programa de control también se necesita Gazebo Harmonic con las bibliotecas `gz-transport13` y `gz-msgs10` (si tienes otras versiones, cambia los nombres de los paquetes en `CMakeLists.txt`).

Desde la carpeta del proyecto:

```
mkdir -p build && cd build
cmake ..
make
cd ..
```

Esto genera `build/control_bst` (si se encontró Gazebo) y `build/pruebas_datos`.

**Pruebas sin Gazebo** (cargan archivos, ordenan, buscan y usan el BST). Desde la carpeta del proyecto:

`g++ -std=c++17 pruebas/pruebas_datos.cc -I. -o pruebas_datos && ./pruebas_datos`

## Instrucciones para ejecutar el avance de proyecto

Ejecuta ambos comandos desde la carpeta del proyecto (las rutas `datos/...` son relativas).

En una primera terminal, abre el mundo (inicia la simulación en marcha con `-r`):

`gz sim -r worlds/brazo_cubos.sdf`

En una segunda terminal, ejecuta el control:

`./build/control_bst`

Opcionalmente se pueden indicar otras rutas: `./build/control_bst otro_catalogo.csv otros_pesos.csv`.

Si existe `datos/pesos.csv`, el programa pregunta si se carga en lugar de escanear. Si no, escanea los 5 cubos (aproximadamente 1.5 minutos) y después muestra el menú.

## Descripción de las entradas del avance de proyecto

El programa recibe cuatro tipos de entrada:

1. **`datos/cubos.csv` (catálogo, obligatorio).** Texto separado por comas, una fila por cubo: `id,color,q1_rad,r_m,peso_real_g`. Las líneas vacías, las que empiezan con `#` y el encabezado se ignoran. Los `id` deben ser consecutivos desde 1, y las posiciones deben coincidir con las del archivo de mundo. Ejemplo (es el archivo incluido):

```
# id,color,q1_rad,r_m,peso_real_g
1,rojo,-0.6,0.40,300
2,azul,-0.3,0.46,60
3,amarillo,0.0,0.42,450
4,verde,0.3,0.46,120
5,morado,0.6,0.40,200
```

2. **`datos/pesos.csv` (pesos medidos, opcional).** Una fila por cubo: `id,peso_g`. Lo genera el programa tras un escaneo (o con la opción 11) y se puede cargar al iniciar o con la opción 12. La posición `q1`, `r` de cada cubo se toma del catálogo, no de este archivo. Ejemplo:

```
id,peso_g
1,301.20
2,59.10
3,449.50
4,120.70
5,199.90
```

3. **Archivo de mundo `worlds/brazo_cubos.sdf` (XML/SDF) y flujo del sensor F/T.** El SDF define el brazo, el sensor, los pedestales y los cubos con su masa; lo lee Gazebo al arrancar. Del tópico `/wrist_ft` (mensajes `gz::msgs::Wrench` a 100 Hz) se toman las componentes de fuerza `fx, fy, fz`, que se promedian durante 1.5 s para cancelar la vibración.
4. **Entradas del usuario en el menú** (teclado): una opción numérica del 0 al 12 y, según la opción, un ID de cubo, un peso en gramos o un rango de pesos (mínimo y máximo).

## Descripción de las salidas del avance de proyecto

* Al iniciar: número de cubos leídos del catálogo, o un mensaje con la línea y el motivo si el archivo es inválido.
* Durante el escaneo, por cada cubo: la tara (`|F0|` en N), la fuerza con el cubo, `|F - tara|` y el peso medido en gramos. Al final, el archivo `datos/pesos.csv` actualizado.
* Al terminar el escaneo (o al cargar pesos), el tamaño y la altura del árbol y una tabla ordenada de menor a mayor peso (Pos, Cubo, Color, Peso medido, Peso real, Error, q1, r). Ejemplo con los pesos de arriba:

```
  Pos  Cubo  Color      Peso medido   Real     Error    q1      r
  ---------------------------------------------------------------------
    0     2  azul            59.1 g   60.0 g   -0.9 g  -0.30  0.46
    1     4  verde          120.7 g  120.0 g   +0.7 g   0.30  0.46
    2     5  morado         199.9 g  200.0 g   -0.1 g   0.60  0.40
    3     1  rojo           301.2 g  300.0 g   +1.2 g  -0.60  0.40
    4     3  amarillo       449.5 g  450.0 g   -0.5 g   0.00  0.42
```

* Opción 3: el árbol dibujado de lado y los cuatro recorridos (preorden, inorden, postorden y por niveles). Como los cubos se insertan en el orden de su ID (300, 60, 450, 120, 200 g nominales), la raíz es el cubo rojo y el árbol tiene altura 4.
* Opciones 4 a 7: mensajes del cubo encontrado y movimiento del brazo en Gazebo, que levanta el cubo, vuelve a pesarlo y lo devuelve a su pedestal al presionar Enter.
* Opción 8: lista de cubos dentro del rango de peso pedido.
* Opción 9: nivel del cubo en el árbol y su lista de ancestros desde la raíz.
* Opción 10: las lecturas ordenadas con Merge Sort, si coinciden con el recorrido inorden del BST, y la posición que devuelve la Búsqueda Binaria para un peso dado junto con el resultado de `find` del BST. Ejemplo:

```
Lecturas ordenadas con Merge Sort:
  C2 59.1g  C4 120.7g  C5 199.9g  C1 301.2g  C3 449.5g
Coincide con el recorrido inorden del BST: si
Busqueda binaria: posicion 2 -> cubo 5 (morado, 199.9 g)
BST (find):       cubo 5
```

* Opciones 11 y 12: confirmación de que los pesos se guardaron en `datos/pesos.csv`, o se cargaron (con la tabla resultante); si el archivo es inválido se informa el error y el árbol actual no se modifica.
* Al salir (opción 0), el brazo regresa a la posición *home*.
* Las pruebas (`pruebas_datos`) imprimen `TODAS LAS PRUEBAS PASARON` o la línea de cada fallo.

## Desarrollo de competencias

### SICT0301: Evalúa los componentes

#### Hace un análisis de complejidad correcto y completo para los algoritmos de ordenamiento usados en el programa.

Considero que he desarrollado esta competencia porque analicé la **complejidad temporal** y **espacial** de los algoritmos de `Sorts<T>` (`sorts.h`), que el programa ejecuta en la opción 10 de `control_bst.cc` (`ordenarYBuscar`) sobre el `vector<Cubo>` de lecturas. Sea n el número de lecturas.

* **Merge Sort (`ordenaMerge`, `mergeSplit`, `mergeArray`, `copyArray`):**
  * **Complejidad temporal:** Θ(n log n) en el mejor, promedio y peor caso. `mergeSplit` divide el vector a la mitad en log₂ n niveles de recursión y `mergeArray` hace a lo más n comparaciones y n copias por nivel (la mezcla y `copyArray` recorren el segmento una vez cada una), es decir, T(n) = 2T(n/2) + Θ(n).
  * **Complejidad espacial:** O(n) auxiliar por el vector temporal `tmp` que crea `ordenaMerge`, más O(log n) de pila de recursión; en total O(n).
  * **Estabilidad:** es estable, porque `mergeArray` toma primero el elemento izquierdo cuando son iguales (`<=`).
* **Búsqueda Binaria (`busqBinaria`):**
  * **Complejidad temporal:** O(log n) en el peor y promedio de los casos; O(1) en el mejor (el elemento está en la posición central inicial). Requiere el vector ordenado, por eso se aplica después de Merge Sort.
  * **Complejidad espacial:** O(1), pues es iterativa y solo usa `low`, `high` y `mitad`.
* **Costo total de la opción 10:** copiar el vector O(n) + Merge Sort Θ(n log n) + recolectar el recorrido inorden del BST O(n) y compararlo con el vector ordenado O(n) + Búsqueda Binaria O(log n) + `find` en el BST O(h). Domina Θ(n log n); con n = 5 el costo es despreciable frente al movimiento del brazo.
* **Observación sobre la tolerancia:** `operator==` compara con tolerancia de 5 g, por lo que la Búsqueda Binaria devuelve *algún* cubo dentro de ±5 g. Esto es correcto porque los pesos del experimento (60, 120, 200, 300 y 450 g) están separados por mucho más que la tolerancia.

#### Hace un análisis de complejidad correcto y completo de todas las estructuras de datos y cada uno de sus usos en el programa.

Considero que he desarrollado esta competencia porque analicé la complejidad temporal y espacial de cada operación de `BST<T>` en `bst.h`, de las funciones de lectura de archivos de `cubo.h` y de cada punto del programa (`control_bst.cc`) donde se usan. Sea **n** el número de cubos, **h** la altura del árbol y **k** el número de resultados de una consulta por rango.

**Altura del BST.** El árbol no se auto-balancea. Con inserciones en orden aleatorio h ≈ O(log n) (mejor y caso promedio), pero si los pesos llegaran en orden creciente o decreciente el árbol degenera en una lista y h = O(n) (peor caso). Por eso las operaciones se expresan en función de h.

**Operaciones de `BST<T>` (archivo `bst.h`)**

| Operación | Tiempo | Espacio auxiliar | Justificación |
|---|---|---|---|
| `add` | O(h): O(log n) mejor/promedio, O(n) peor | O(1) | Descenso iterativo hasta la hoja libre; crea un nodo. |
| `find` | O(h): O(1) mejor (está en la raíz), O(log n) promedio, O(n) peor | O(1) | Descenso iterativo comparando con `<` y `==`. |
| `minimum`, `maximum` | O(h) | O(1) | Se baja siempre por el hijo izquierdo (o derecho). |
| `whatlevelamI` | O(h) | O(1) | Descenso iterativo contando niveles. |
| `ancestors` | O(h) | O(h) por la cadena de salida | Dos descensos desde la raíz (uno para verificar y otro para imprimir). |
| `range` | O(h + k) | O(h) pila de recursión | Poda las ramas fuera de [lo, hi]; solo visita los nodos del rango y los caminos hacia ellos. |
| `inorderApply` | Θ(n) | O(h) pila de recursión | Visita todos los nodos una vez. |
| `findIf` | Θ(n) siempre | O(h) | Se apoya en `inorderApply`, que no se detiene al encontrar el elemento. |
| `height` | Θ(n) | O(h) | Recorre todos los nodos. |
| `visit` | Θ(n) | O(n) | Hace 4 recorridos (pre, in, post y por niveles); los strings de salida y la cola `std::queue` del recorrido por niveles ocupan hasta O(n). |
| `printTree` | Θ(n) | O(h) | Un recorrido derecha-raíz-izquierda. |
| `clear` y destructor | Θ(n) | O(h) | Borrado en postorden de todos los nodos. |
| `size`, `empty` | O(1) | O(1) | Contador `count` y comparación de la raíz. |

El árbol completo ocupa O(n) de memoria (un `Node` por cubo). Se prohibió copiar el árbol para evitar copias superficiales de punteros.

**Lectura y escritura de archivos (`cubo.h`)**. Sea L la longitud máxima de una línea.

| Función | Tiempo | Espacio | Justificación |
|---|---|---|---|
| `csv::leerLineas` | O(n·L) | O(n·L) | Lee el archivo una vez y guarda solo las líneas de datos. |
| `cargarCatalogo` | O(n·L) | O(n·L) | Una pasada con validación O(1) por campo; el `id` debe ser igual a la posición, sin búsquedas. |
| `cargarPesos` | O(n log n + n·L) | O(n) | Cada fila se inserta en un `std::set<int>` (árbol balanceado, O(log n)) para detectar ids repetidos. |
| `guardarPesos` | O(n) | O(1) | Una escritura por cubo. |

**Usos en `control_bst.cc`**

| Uso en el programa | Costo |
|---|---|
| Inicio: `cargarCatalogo` | O(n·L), una sola vez. Llena `INFO` (`vector<InfoCubo>`), cuyo acceso por `id` es O(1). |
| Opción 12 / inicio con pesos guardados: `cargarPesos` + `clear` + n × `add` | O(n log n + n·L) + O(n·h): O(n log n) promedio, O(n²) peor. |
| `escanear`: `clear` + n × `add` + `push_back` al vector + `guardarPesos` | O(n·h) en el árbol (O(n log n) promedio, O(n²) peor) y O(n) amortizado en el vector. El tiempo real lo domina el movimiento físico del brazo, que es Θ(n) tomas de varios segundos cada una. |
| `imprimirTabla` (opción 2), `inorderApply` | Θ(n) |
| Opción 3: `printTree` + `visit` + `height` + `size` | Θ(n) tiempo, O(n) espacio por `visit` |
| Opciones 4 y 9 (buscar por ID con `findIf`) | Θ(n). El árbol está ordenado por peso, no por ID, así que no se puede podar la búsqueda. Con n = 5 es despreciable. |
| Opción 5 (buscar por peso con `find`) | O(h). Si no hay coincidencia, se busca el más cercano con `inorderApply`: Θ(n). |
| Opciones 6 y 7 (`minimum`, `maximum`) | O(h) |
| Opción 8 (`range`) | O(h + k) |
| Opción 9 (`whatlevelamI`, `ancestors`) | O(h) |
| Opción 10 (`ordenarYBuscar`) | Θ(n log n) tiempo y O(n) espacio (ver el análisis de Merge Sort). |
| Opción 11 (`guardarPesos`) | O(n) |

**Otras estructuras utilizadas**

* `std::vector<Cubo>` (`lecturas`) y `std::vector<InfoCubo>` (`INFO`): arreglos dinámicos; acceso por índice O(1), `push_back` O(1) amortizado, espacio O(n).
* `std::array` (`Q` de 4 articulaciones y `Vec3` de la fuerza): tamaño fijo, acceso O(1), espacio O(1) respecto a n.
* `std::map<std::string, Publisher>` (`pubs_`, 6 entradas): acceso O(log m) con m = 6 (constante en la práctica), espacio O(m).
* `std::map<int, Publisher>` (`attachPub_`, `detachPub_`): solo se llenan si `USAR_ATTACH` es verdadero (en esta versión está desactivado).
* Funciones del brazo: `IK` es O(1) (fórmulas cerradas); `IrJoints`, `MoverGripper` y `LineaVertical` hacen un número de pasos proporcional a la duración (seg × 50), independiente de n; `PromediarFuerza` es O(T) con T = ventana de muestreo (1.5 s) y espacio O(1) porque acumula una suma en lugar de guardar las muestras.

### SICT0302: Toma decisiones

#### Selecciona un algoritmo de ordenamiento adecuado al problema y lo usa correctamente.

Considero que he desarrollado esta competencia porque elegí Merge Sort y Búsqueda Binaria según las restricciones del problema y los uso en `control_bst.cc` (función `ordenarYBuscar`) sobre el vector de lecturas de peso.

* **Por qué Merge Sort:** garantiza Θ(n log n) en todos los casos (no hay un peor caso cuadrático como en Quick Sort con malos pivotes ni como en Insertion o Selection Sort), es **estable** y trabaja bien sobre un vector. En un sistema robótico es preferible una respuesta determinista antes de enviar datos a los actuadores. Su costo es O(n) de memoria auxiliar, aceptable con lotes pequeños y que se controla porque el tamaño del vector es el número de cubos.
* **Por qué Búsqueda Binaria:** una vez ordenado el vector, permite verificar si hay una lectura de cierto peso en O(log n) con espacio O(1). Se eligió la versión **iterativa** para no consumir pila de recursión, lo cual importa en microcontroladores.
* **Uso correcto:** se ordena una *copia* del vector de lecturas (el original conserva el orden de medición), la búsqueda se hace solo sobre el vector ya ordenado, se maneja el vector vacío (`ordenaMerge` y `busqBinaria` regresan sin fallar y la función avisa que no hay lecturas) y el resultado se **valida** comparándolo con el recorrido inorden del BST. Las pruebas de `pruebas/pruebas_datos.cc` verifican el orden esperado (60, 120, 200, 300, 450 g), una búsqueda exacta, una dentro de la tolerancia de 5 g, una que no existe y el vector vacío.
* **Relación con el BST:** el BST mantiene el orden de forma incremental mientras llegan las lecturas; Merge Sort ordena un lote completo de una vez. Tener ambos permite contrastar los dos enfoques con los mismos datos.

#### Selecciona una estructura de datos adecuada al problema y la usa correctamente.

Considero que he desarrollado esta competencia porque elegí el BST comparando alternativas según las operaciones que el problema exige. La evidencia está en `bst.h` y en el uso del árbol en `control_bst.cc`.

* **Qué necesita el problema:** guardar pocos pares (peso, posición) que se obtienen de uno en uno, listarlos de menor a mayor, encontrar el más ligero y el más pesado, buscar por peso con tolerancia, listar un rango de pesos y repetir el escaneo en cualquier momento.
* **Alternativas descartadas:**
  * *Vector ordenado + búsqueda binaria* (avance 1): búsqueda O(log n) pero cada inserción cuesta O(n) por el desplazamiento de elementos, y hay que reordenar en cada re-escaneo. Se conserva solo como vector auxiliar de lecturas.
  * *Tabla hash* (`unordered_map`): búsqueda exacta en O(1), pero no mantiene el orden, no da mínimo/máximo/rango sin recorrer todo, y no sirve con igualdad por tolerancia (dos pesos "casi iguales" caen en cubetas distintas).
  * *Lista enlazada:* búsqueda y orden O(n).
* **Por qué el BST:** la inserción, la búsqueda, el mínimo, el máximo y el rango tienen costo O(h) (O(log n) en promedio), el orden sale gratis con el recorrido inorden, y la comparación con `operator<` y `operator==` con tolerancia de 5 g permite buscar un peso medido con ruido. Los pesos de los cubos (60, 120, 200, 300 y 450 g) están separados por mucho más que la tolerancia, por lo que la comparación es consistente.
* **Otras decisiones:** `std::vector` para el catálogo (`INFO`) y las lecturas, porque el tamaño es pequeño, el acceso por `id - 1` es O(1) y se recorre en orden; `std::set<int>` solo dentro de `cargarPesos` para detectar ids repetidos en O(log n) por fila.
* **Uso correcto:** el árbol se limpia (`clear`) antes de cada re-escaneo o carga para no duplicar cubos, administra su memoria con `deleteAll` en el destructor, regresa punteros `const T*` (o `nullptr` cuando no hay resultado) que el menú siempre revisa antes de usar, y se prohíbe su copia. Si una carga de archivo falla, el árbol anterior no se modifica.
* **Limitación reconocida:** el BST no se auto-balancea. Con n = 5 la altura es 4 como máximo y no afecta, pero para lotes grandes de pesos ya ordenados convendría un AVL o un árbol rojinegro para garantizar O(log n). Se priorizó mantener una implementación sencilla y comprensible.

### SICT0303: Implementa acciones científicas

#### Implementa mecanismos para consultar información de las estructuras correctos.

Considero que he desarrollado esta competencia porque el menú de `control_bst.cc` expone consultas distintas, cada una implementada con el método del árbol que mejor aprovecha su estructura:

* **Por peso (opción 5):** `find` con igualdad con tolerancia. Si no hay coincidencia, se recorre el árbol para ofrecer el cubo más cercano en peso y se pide confirmación antes de levantarlo.
* **Mínimo y máximo (opciones 6 y 7):** `minimum` y `maximum`, que solo bajan por una rama del árbol.
* **Por rango (opción 8):** `range`, que poda los subárboles que no pueden contener valores del intervalo.
* **Por ID (opción 4):** `findIf` con un predicado (lambda).
* **Nivel y ancestros (opción 9):** `whatlevelamI` y `ancestors`.
* **Listado ordenado y recorridos (opciones 2 y 3):** `inorderApply`, `visit` (preorden, inorden, postorden y por niveles) y `printTree`.
* **Búsqueda en el vector ordenado (opción 10):** `busqBinaria`, contrastada con `find` del BST.

Las consultas validan sus entradas: la lectura numérica (`leerNumero`) atrapa entradas no numéricas con `try/catch` y regresa `NAN`; las opciones rechazan IDs o pesos inválidos; el rango de IDs se toma del catálogo (`nCubos()`); y el árbol vacío se maneja explícitamente (`empty`, `nullptr`).

#### Implementa mecanismos de lectura de archivos para cargar datos a las estructuras de manera correcta.

Considero que he desarrollado esta competencia porque el programa carga sus datos desde archivos de texto con `std::ifstream` y los vuelca a las estructuras con validación. La implementación está en `cubo.h` y se usa desde `main` y desde el menú de `control_bst.cc`.

* **Catálogo (`cargarCatalogo`, archivo `datos/cubos.csv`):** lee el archivo línea por línea, ignora líneas vacías, comentarios (`#`) y el encabezado, separa los campos por coma y llena el `vector<InfoCubo>` `INFO`, del que se obtienen el color, la posición (`q1`, `r`) y el peso nominal de cada cubo. Esto sustituye la tabla que antes estaba escrita en el código.
* **Pesos (`cargarPesos`, archivo `datos/pesos.csv`):** lee los pares `id,peso_g` y construye objetos `Cubo` (tomando `q1` y `r` del catálogo) que se insertan en el BST con `add` y en el vector de lecturas. Es el complemento de `guardarPesos`, que escribe el mismo formato después de un escaneo, de modo que el programa puede arrancar sin repetir los ~1.5 minutos de medición.
* **Validación (manejo de errores):** se verifica que el archivo exista y se pueda abrir; que cada línea tenga el número esperado de campos; que los números se conviertan *completos* (se rechaza `300x` o un campo vacío) con `std::stoi`/`std::stod` dentro de `try/catch`; que los `id` del catálogo sean consecutivos desde 1; que no haya ids repetidos ni fuera del catálogo en los pesos; y que `r`, `peso` y `q1` estén en rango. Cada error indica **la línea y el motivo**. Si la carga falla, las estructuras existentes no se modifican (se construye un vector temporal y solo se copia al terminar bien).
* **Complemento:** el flujo del sensor `/wrist_ft` (suscripción con `node_.Subscribe` y callback `OnWrench` protegido con `mutex`) alimenta el peso medido, y el archivo de mundo `brazo_cubos.sdf` es leído por Gazebo.
* **Evidencia de pruebas:** `pruebas/pruebas_datos.cc` carga el catálogo real, comprueba que se rechazan archivos inexistentes, con campos de más o de menos, ids no consecutivos, números inválidos y valores fuera de rango, que un error no modifica la salida, y que guardar y volver a cargar los pesos devuelve los mismos valores.
