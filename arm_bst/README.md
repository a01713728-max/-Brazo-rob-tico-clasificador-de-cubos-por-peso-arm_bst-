# arm_bst: clasificación de cubos por peso con un brazo robótico, un sensor F/T y un árbol binario de búsqueda

**Resumen.** Este proyecto simula en Gazebo Harmonic un brazo robótico de 4 grados de libertad (GDL) con un gripper tipo jaula y un sensor de fuerza/torque (F/T) en la muñeca. El brazo levanta cinco cubos de 60 a 450 g, estima la masa de cada uno a partir de la fuerza medida (descontando una tara), y almacena las lecturas en un árbol binario de búsqueda (BST) ordenado por peso. Sobre esa estructura se ofrece un menú de consultas (mínimo, máximo, rango, nivel, ancestros, recorridos), una verificación cruzada con Merge Sort y Búsqueda Binaria, y la orden física de levantar el cubo seleccionado. El catálogo de cubos y los pesos medidos se leen y escriben en archivos CSV validados. En el caso de prueba, el error de medición reportado fue de ±1.2 g como máximo (ver §5).

---

## Inicio rápido

Desde la carpeta del proyecto, en Ubuntu (o WSL2 con WSLg):

```bash
# 1) Compilar
mkdir -p build && cd build && cmake .. && make && cd ..

# 2) Terminal 1: abrir la simulación
gz sim -r worlds/brazo_cubos.sdf

# 3) Terminal 2: ejecutar el control
./build/control_bst
```

Si existe `datos/pesos.csv`, el programa pregunta si se cargan esos pesos en lugar de escanear (responde `s` para arrancar al instante). Si no, escanea los 5 cubos (aprox. 2 a 2.5 min) y abre el menú. Para salir usa la opción `0`: el brazo regresa a su posición *home*.

Pruebas sin Gazebo:

```bash
g++ -std=c++17 pruebas/pruebas_datos.cc -I. -o pruebas_datos && ./pruebas_datos
```

Resultado esperado: `TODAS LAS PRUEBAS PASARON`.

**Requisitos:** compilador C++17, CMake y, para el control, Gazebo Harmonic con `gz-transport13` y `gz-msgs10` (si usas otras versiones, cambia los nombres de paquete en `CMakeLists.txt`).

**Problemas comunes**

| Síntoma | Causa probable | Solución |
|---|---|---|
| `No se pudo cargar el catalogo: no se pudo abrir 'datos/cubos.csv'` | Se ejecutó fuera de la carpeta del proyecto | `cd` a la carpeta del proyecto o pasar la ruta como argumento |
| `No llegan datos de /wrist_ft` | La simulación no está corriendo o está en pausa | Abrir el mundo con `-r` y verificar con `gz topic -l` |
| `Punto inalcanzable` | Posición del cubo fuera del alcance del brazo | Revisar `q1` y `r` en `datos/cubos.csv` |
| Gazebo no abre ventana en WSL | WSLg no activo | Verificar `echo $DISPLAY` (debe devolver algo como `:0`) |

---

## 1. Introducción y objetivo

Clasificar objetos por peso es una tarea común en líneas de producción. Aquí se plantea una versión reducida: ¿cómo organizar lecturas de peso obtenidas **una por una** de un sensor físico para que puedan consultarse eficientemente y para que el robot pueda volver a cada objeto?

El proyecto evolucionó a partir de un primer avance en el que un `vector<float>` de pesos simulados se ordenaba una sola vez con Merge Sort y se consultaba con Búsqueda Binaria (`sorts.h`). Ese enfoque supone un lote estático. En este segundo avance:

1. Los pesos dejan de ser simulados y se **miden** con el sensor F/T.
2. Cada lectura se asocia a una posición (`q1`, `r`) para que el brazo pueda regresar por el cubo.
3. El lote se vuelve dinámico (se puede re-escanear), por lo que se adopta un **BST** que mantiene el orden durante la inserción.
4. Los datos se **externalizan en archivos CSV**, de modo que cambiar un cubo no exige recompilar y no hay que repetir el escaneo en cada ejecución.

## 2. Arquitectura

```
arm_bst/
├── CMakeLists.txt
├── README.md
├── control_bst.cc          control del brazo, escaneo, menú y consultas
├── bst.h                   plantilla BST<T>
├── sorts.h                 plantilla Sorts<T> (Merge Sort y Búsqueda Binaria, del avance 1)
├── cubo.h                  Cubo / InfoCubo y lectura/escritura de CSV
├── datos/
│   └── cubos.csv           catálogo (entrada). pesos.csv se genera al escanear
├── worlds/
│   └── brazo_cubos.sdf     mundo de Gazebo: brazo, sensor F/T, pedestales y cubos
└── pruebas/
    └── pruebas_datos.cc    pruebas de archivos, sorts y BST (sin Gazebo)
```

Flujo general:

```
cubos.csv --> cargarCatalogo --> INFO (vector<InfoCubo>)
                                   |
   [pesos.csv --> cargarPesos] ----+--> BST<Cubo> + vector<Cubo> lecturas
   [o escaneo con sensor F/T]                |
                                             +--> menú: consultas, Merge Sort/Bús. Binaria, levantar cubo
                                             +--> guardarPesos --> pesos.csv
```

## 3. Métodos

### 3.1 Medición de masa con el sensor F/T

Para cada cubo el brazo viaja a su vertical mediante cinemática inversa plana (fórmula cerrada) y realiza:

1. **Tara.** Con el gripper cerrado y vacío, promedia el vector de fuerza durante 1.5 s (`PromediarFuerza`). El promedio del *vector* (no de su norma) cancela la vibración, que cambia de signo.
2. **Agarre por jaula.** Baja, cierra el gripper alrededor del cubo y lo levanta. El gripper no aprieta: los labios inferiores quedan bajo el cubo y las placas laterales dejan 1 a 2 mm de holgura, de modo que el sensor registra el peso real que carga la mano. La constante `USAR_ATTACH` es `false` (no se suelda el cubo).
3. **Medición.** Con la mano cargada vuelve a promediar la fuerza y calcula

   `masa = |F_con_cubo − F_tara| / g`  (con g = 9.81 m/s²)

   La resta de la tara elimina el peso del propio gripper y cualquier *offset* constante del sensor.
4. El cubo se devuelve a su pedestal y la lectura `Cubo{id, peso, q1, r}` se inserta en el BST y en un vector de lecturas.

Las rampas suaves (`s²(3−2s)`) en articulaciones y dedos evitan que el gripper golpee y desplace los cubos.

### 3.2 Estructura de datos principal: BST de `Cubo`

El BST ordena por `peso` (`operator<`). La igualdad (`operator==`) tiene **tolerancia de 5 g**, porque un peso medido nunca es exacto. Esto es consistente mientras los pesos distintos difieran más que la tolerancia (aquí: 60, 120, 200, 300 y 450 g). El árbol administra memoria dinámica, por lo que se prohíbe su copia.

Métodos añadidos al BST original para el menú: `find`, `findIf`, `minimum`, `maximum`, `range`, `inorderApply`, `printTree`, `size`, `clear`.

### 3.3 Verificación cruzada con `Sorts<T>`

La opción 10 ordena una *copia* del vector de lecturas con Merge Sort y comprueba que coincide con el recorrido inorden del BST; luego localiza un peso con Búsqueda Binaria y lo contrasta con `find`. Dos mecanismos independientes se validan entre sí con los mismos datos.

### 3.4 Archivos CSV

| Archivo | Formato | Lectura | Escritura |
|---|---|---|---|
| `datos/cubos.csv` (obligatorio) | `id,color,q1_rad,r_m,peso_real_g` | `cargarCatalogo` | manual |
| `datos/pesos.csv` (opcional) | `id,peso_g` | `cargarPesos` | `guardarPesos` |

Ejemplo de catálogo (`#` = comentario; también se ignoran líneas vacías y el encabezado):

```
# id,color,q1_rad,r_m,peso_real_g
1,rojo,-0.6,0.40,300
2,azul,-0.3,0.46,60
3,amarillo,0.0,0.42,450
4,verde,0.3,0.46,120
5,morado,0.6,0.40,200
```

**Lectura.** Usa `std::ifstream`, separa campos por coma y convierte el texto **completo** con `stoi`/`stod` en `try/catch` (se rechaza `300x` o un campo vacío). Valida: número de campos; `id` consecutivos desde 1; `r > 0`, `peso > 0`, `|q1| ≤ 3.14`; ids repetidos o fuera del catálogo en los pesos. Cada error indica **la línea y el motivo**. Si la carga falla, las estructuras existentes **no se modifican** (se construye un vector temporal y se copia solo al terminar bien). Si el catálogo es inválido, el programa termina antes de mover el brazo. La posición `q1`, `r` de cada cubo se toma siempre del catálogo, nunca de `pesos.csv`, para evitar inconsistencias.

**Escritura.** `guardarPesos` escribe `datos/pesos.csv` tras cada escaneo y con la opción 11. Escribe primero a un archivo temporal (`pesos.csv.tmp`), verifica el estado del flujo tras `flush()` y solo entonces lo renombra, de modo que un fallo a la mitad **no destruye** los pesos anteriores. Rechaza un vector vacío y reporta el error al llamador con un mensaje. Los pesos se guardan **en orden de `id`** (el orden de inserción del escaneo): al recargarlos, `cargarPesos` reconstruye el mismo BST (misma forma y altura). Guardarlos en orden de peso degeneraría el árbol en una lista. Los pesos se redondean a 2 decimales (±0.005 g).

```
id,peso_g
1,301.20
2,59.10
3,449.50
4,120.70
5,199.90
```

## 4. Análisis de complejidad

Notación: n = número de cubos; h = altura del BST; k = resultados de una consulta por rango; F = líneas del archivo (F = O(n)); L = longitud de una línea; q = consultas en una sesión.

### 4.1 Ordenamiento y búsqueda (`sorts.h`)

| Algoritmo | Tiempo | Espacio | Notas |
|---|---|---|---|
| Merge Sort (`ordenaMerge`) | Θ(n log n) en todos los casos | O(n) (vector `tmp`) + O(log n) de pila | T(n) = 2T(n/2) + Θ(n). Estable: `mergeArray` toma primero el izquierdo en empates (`<=`). |
| Búsqueda Binaria (`busqBinaria`) | O(log n) peor/promedio; O(1) mejor | O(1) (iterativa) | Requiere vector ordenado. Con la tolerancia de 5 g devuelve *algún* cubo dentro de ±5 g. |

### 4.2 Estructuras de datos y sus usos

El BST no se auto-balancea: h = O(log n) con inserciones en orden aleatorio, pero h = O(n) si los pesos llegan ordenados.

| Operación (`bst.h`) | Tiempo | Espacio aux. |
|---|---|---|
| `add` | O(h) | O(1) |
| `find`, `minimum`, `maximum`, `whatlevelamI` | O(h) | O(1) |
| `ancestors` | O(h) | O(h) |
| `range` | O(h + k) | O(h) |
| `inorderApply`, `findIf`, `height`, `printTree`, `clear` | Θ(n) | O(h) |
| `visit` (4 recorridos) | Θ(n) | O(n) |
| `size`, `empty` | O(1) | O(1) |

| Uso en el programa | Costo |
|---|---|
| Inicio: `cargarCatalogo` | O(F·L) |
| `escanear` / `cargarPesos` + n × `add` | O(n·h): O(n log n) promedio, O(n²) peor |
| Opciones 2 y 3 (tabla, árbol, recorridos) | Θ(n) |
| Opciones 4 y 9 (buscar por ID con `findIf`) | Θ(n): el árbol está ordenado por peso, no por ID |
| Opción 5 (por peso con `find`) | O(h); si no hay coincidencia, Θ(n) para el más cercano |
| Opciones 6 y 7 (mínimo y máximo) | O(h) |
| Opción 8 (rango) | O(h + k) |
| Opción 9 (`whatlevelamI`, `ancestors`) | O(h) |
| Opción 10 (Merge Sort + Bús. Binaria + `find`) | Θ(n log n) tiempo, O(n) espacio |
| Opciones 11 y 12 (`guardarPesos`, `cargarPesos`) | O(n) y O(n log n + F·L) (un `std::set<int>` detecta ids repetidos) |

Otras estructuras: `vector<Cubo>` y `vector<InfoCubo>` (acceso O(1), `push_back` O(1) amortizado, espacio O(n)); `std::array` para `Q` y `Vec3` (tamaño fijo); `std::map` de 6 publicadores (acceso constante en la práctica).

### 4.3 Resto de los componentes

* **Procesamiento de texto (`cubo.h`).** `recortar`, `separar`, `aEntero` y `aDouble` son O(L) por línea; `leerLineas` es O(F·L). Los operadores de `Cubo` son O(1).
* **Comunicación con Gazebo.** `MoverJoint`, `Gripper`, `Enviar` y el callback `OnWrench` (100 Hz, copia 3 números bajo un `mutex`) son O(1).
* **Cinemática.** `IK` es O(1) (fórmula cerrada).
* **Movimiento.** `IrJoints`, `LineaVertical`, `MoverGripper`, `ViajarA` e `Home` ejecutan P ≤ 300 pasos de 20 ms. P depende de la duración configurada y no de n, así que son O(1) respecto a n.
* **Sensor.** `PromediarFuerza` acumula ≈150 muestras (1.5 s a 10 ms) con espacio O(1), sin guardarlas. `MedirTara` y `MedirMasaGramos` son O(1) respecto a n.
* **Rutinas compuestas.** `TomarCubo`, `SoltarCubo` y `levantar` son O(1) respecto a n (≈26 s de tiempo físico por cubo). `escanear` ejecuta n ciclos: Θ(n) acciones físicas más O(n·h) de cómputo.
* **Consola.** `leerLinea` y `leerNumero` son O(longitud de la entrada); `menu` es O(1).

### 4.4 Complejidad final

| Métrica | Resultado |
|---|---|
| Tiempo de cómputo de una sesión | O(n·h + q·n log n). Promedio: O(n log n) por operación dominante (opción 10). Peor caso: O(n²) por la construcción de un BST degenerado. |
| Espacio | O(n): catálogo, lecturas, nodos del BST y auxiliares de la opción 10. La pila de recursión del BST es O(h). |
| Tiempo real de ejecución | Θ(n), dominado por el movimiento del brazo (≈26 s por cubo). |

Con n = 5 y h = 4 el cómputo es de microsegundos frente a más de dos minutos de escaneo, por lo que el rendimiento real lo fija la física del brazo y no los algoritmos.

## 5. Decisiones de diseño y resultados

**Por qué un BST.** El problema exige insertar lecturas de una en una, listarlas de menor a mayor, obtener mínimo y máximo, buscar con tolerancia, consultar rangos y re-escanear en cualquier momento.

| Alternativa | Motivo de descarte |
|---|---|
| Vector ordenado + búsqueda binaria | Búsqueda O(log n), pero inserción O(n) y reordenamiento en cada re-escaneo. Se conserva como vector auxiliar de lecturas. |
| Tabla hash | Búsqueda exacta O(1), pero sin orden, sin mínimo/máximo/rango y sin igualdad por tolerancia. |
| Lista enlazada | Búsqueda y orden O(n). |

**Por qué Merge Sort y Búsqueda Binaria.** Merge Sort garantiza Θ(n log n) sin peor caso cuadrático, es estable y su costo de memoria es aceptable para lotes pequeños. La Búsqueda Binaria es iterativa (O(1) de espacio, sin pila de recursión).

**Resultados.** Con los cubos del catálogo, los pesos medidos fueron:

```
  Pos  Cubo  Color      Peso medido   Real     Error    q1      r
  ---------------------------------------------------------------------
    0     2  azul            59.1 g   60.0 g   -0.9 g  -0.30  0.46
    1     4  verde          120.7 g  120.0 g   +0.7 g   0.30  0.46
    2     5  morado         199.9 g  200.0 g   -0.1 g   0.60  0.40
    3     1  rojo           301.2 g  300.0 g   +1.2 g  -0.60  0.40
    4     3  amarillo       449.5 g  450.0 g   -0.5 g   0.00  0.42
```

El error absoluto máximo es 1.2 g (0.4 % en el cubo de 300 g), muy por debajo de la tolerancia de 5 g. Como los cubos se insertan en orden de ID (300, 60, 450, 120, 200 g), la raíz es el cubo rojo y el árbol tiene altura 4. La opción 10 produce:

```
Lecturas ordenadas con Merge Sort:
  C2 59.1g  C4 120.7g  C5 199.9g  C1 301.2g  C3 449.5g
Coincide con el recorrido inorden del BST: si
Busqueda binaria: posicion 2 -> cubo 5 (morado, 199.9 g)
BST (find):       cubo 5
```

**Pruebas (`pruebas_datos.cc`).** Verifican: catálogo real; rechazo de archivos inexistentes, con campos de más o de menos, ids no consecutivos, números inválidos y valores fuera de rango; que un error no modifica las estructuras; ciclo guardar → cargar de los pesos; Merge Sort (orden 60, 120, 200, 300, 450 g); Búsqueda Binaria (exacta, dentro de tolerancia, inexistente, vector vacío); y operaciones del BST.

## 6. Uso del programa

### Entradas

1. **`datos/cubos.csv`** (obligatorio): ver §3.4. Las posiciones deben coincidir con las de `worlds/brazo_cubos.sdf`.
2. **`datos/pesos.csv`** (opcional): se carga al iniciar o con la opción 12.
3. **`worlds/brazo_cubos.sdf`** y el tópico `/wrist_ft` (`gz::msgs::Wrench`, 100 Hz): del que se toman `fx, fy, fz`, promediadas 1.5 s.
4. **Teclado**: opción 0 a 12 y, según la opción, un ID, un peso en gramos o un rango.

Se pueden indicar otras rutas: `./build/control_bst otro_catalogo.csv otros_pesos.csv`.

### Menú

| Opción | Acción |
|---|---|
| 1 | Re-escanear y pesar todos los cubos |
| 2 | Listar cubos ordenados por peso (inorden) |
| 3 | Ver el árbol y sus recorridos (preorden, inorden, postorden, por niveles) |
| 4 | Levantar un cubo por ID |
| 5 | Levantar un cubo por peso (`find`; si no hay coincidencia ofrece el más cercano) |
| 6 / 7 | Levantar el más ligero / el más pesado |
| 8 | Listar cubos en un rango de peso |
| 9 | Ver nivel y ancestros de un cubo |
| 10 | Merge Sort + Búsqueda Binaria, contrastados con el BST |
| 11 / 12 | Guardar / cargar pesos (`datos/pesos.csv`) |
| 0 | Salir (el brazo regresa a *home*) |

Las opciones 4 a 7 levantan el cubo, lo vuelven a pesar y lo devuelven a su pedestal al presionar Enter. Las entradas no numéricas o inválidas se rechazan con un mensaje sin cerrar el programa.

### Salidas

* Al iniciar: número de cubos leídos, o la línea y el motivo si el catálogo es inválido.
* Durante el escaneo, por cubo: tara `|F0|`, `|F con cubo|`, `|F − tara|` y peso en gramos; al final, tabla ordenada y `datos/pesos.csv` actualizado.
* Opciones 11 y 12: confirmación de guardado o carga; si el archivo es inválido se informa el error y el árbol actual no cambia.
* Pruebas: `TODAS LAS PRUEBAS PASARON` o la línea de cada fallo.

## 7. Limitaciones y trabajo futuro

* **El BST no se auto-balancea.** Con n = 5 no importa, pero lotes grandes de pesos ordenados llevarían a h = O(n). Un AVL o un árbol rojinegro garantizaría O(log n).
* **Búsquedas por ID** son Θ(n) porque el árbol está ordenado por peso. Un índice auxiliar por ID (un `vector` de punteros) las haría O(1).
* **Tolerancia fija de 5 g.** Es válida solo si los pesos distintos difieren más que ese valor. Con pesos más cercanos habría que reducirla.
* **Simulación, no hardware real.** Las masas, la fricción y el ruido del sensor corresponden al modelo de Gazebo; en un robot real habría que recalibrar la tara y el tiempo de promediado.
* **Duración del escaneo.** Está dominada por esperas fijas para estabilizar el brazo (≈26 s por cubo).

## 8. Ética, uso de herramientas de IA y fuentes

### 8.1 Declaración de uso de IA

Durante este proyecto utilicé un asistente de IA (Claude, de Anthropic) como apoyo y guía, no como sustituto de mi proceso de aprendizaje. Su uso se concentró en:

* **Modelado y archivo de mundo (`worlds/brazo_cubos.sdf`).** Me orientó en la estructura del SDF (enlaces, articulaciones, pedestales, cubos, sensor F/T y plugins de Gazebo) y en la depuración del XML, por ejemplo la etiqueta `<plugin>` sin cerrar que impedía cargar el mundo.
* **Control del brazo.** El ajuste del PID me resultó difícil. Recibí guía para entender las ganancias de los controladores de posición de las articulaciones (`JointPositionController`) y para diseñar la secuencia de movimiento (rampas suaves, esperas de asentamiento, agarre por jaula sin apretar y toma de tara) en `control_bst.cc`.
* **Documentación y análisis.** Recibí ayuda para redactar y organizar este README y para revisar mi análisis de complejidad y la validación de los mecanismos de lectura y escritura de archivos.

Mi trabajo propio: el árbol binario de búsqueda (partí de mi Tarea 3 y le añadí los métodos que necesita el menú), `sorts.h` (Merge Sort y Búsqueda Binaria de mi primer avance), la decisión de estructuras de datos y algoritmos, y la integración del programa. Compilé y probé el código, y revisé las sugerencias de la IA antes de incorporarlas; las que no entendí las consulté en la documentación oficial listada abajo.

### 8.2 Fuentes y bibliotecas

**Bibliotecas utilizadas**

| Biblioteca | Uso |
|---|---|
| Gazebo Harmonic (`gz-sim`) | Simulación física del brazo, los cubos y el sensor |
| `gz-transport13` | Publicar comandos de posición (`/model/arm/joint/<j>/0/cmd_pos`) y suscribirse a `/wrist_ft` |
| `gz-msgs10` | Mensajes `gz::msgs::Double` y `gz::msgs::Wrench` |
| CMake | Compilación |

**Documentación y bibliografía consultadas**

* Gazebo Sim, *Joint Controllers* y referencia de `JointPositionController` (ganancias `p_gain`, `i_gain`, `d_gain`, `cmd_max`, `cmd_min`; tópico `cmd_pos`): <https://gazebosim.org/api/sim/8/jointcontrollers.html>
* Gazebo Sim, sistemas de sensor de fuerza/torque (`ForceTorque`) y de articulación desprendible (`DetachableJoint`, usado en una versión previa del proyecto): <https://gazebosim.org/docs>
* Especificación SDFormat: <http://sdformat.org/spec>
* Repositorio `gazebosim/gz-sim` (ejemplos de mundos SDF con controladores de articulación y sensores): <https://github.com/gazebosim/gz-sim>
* Repositorio `gazebosim/gz-transport` (ejemplos de publicadores y suscriptores en C++): <https://github.com/gazebosim/gz-transport>
* `gazebo_grasp_fix` (plugin de agarre para Gazebo Classic, de J. Buehler): referencia conceptual del agarre por unión fija que en Harmonic se emula con `DetachableJoint`; en la versión final se sustituyó por el agarre por jaula.
* Clase de Eduardo Daniel Juárez Pineda, Tecnológico de Monterrey (ITESM), agosto-diciembre 2026, **[nombre de la materia]**: árboles binarios de búsqueda, Merge Sort, búsqueda binaria y análisis de complejidad.
* MIT OpenCourseWare, **6.006 Introduction to Algorithms** (material de la clase de algoritmos y estructuras de datos): <https://ocw.mit.edu/courses/6-006-introduction-to-algorithms-spring-2020/>
* B. Siciliano et al., *Robotics: Modelling, Planning and Control* (cinemática inversa de un manipulador plano de dos eslabones, usada en `IK`).
* K. J. Åström y R. M. Murray, *Feedback Systems* (control PID).

Sobre el parecido con otros autores: la cinemática inversa plana, el control por posición con PID y el uso de tópicos de `gz-transport` son técnicas estándar documentadas en las fuentes anteriores. No copié código de ningún repositorio; los repositorios listados fueron consulta de ejemplos y documentación.

**Autor:** José Arturo Pérez Cruz (A01713728)
