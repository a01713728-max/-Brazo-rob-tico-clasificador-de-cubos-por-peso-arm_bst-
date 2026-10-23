// pruebas_datos.cc - pruebas SIN Gazebo de: lectura/escritura de CSV (cubo.h), Merge Sort y
// Busqueda Binaria (sorts.h) y el BST (bst.h).
// Compilar y ejecutar desde la carpeta del proyecto:
//   g++ -std=c++17 pruebas/pruebas_datos.cc -I. -o pruebas_datos && ./pruebas_datos
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "bst.h"
#include "cubo.h"
#include "sorts.h"

static int fallos = 0;
#define CHECK(cond)                                                                 \
  do {                                                                              \
    if (!(cond)) { std::cout << "  FALLO linea " << __LINE__ << ": " #cond "\n"; ++fallos; } \
  } while (0)

static void escribir(const std::string &ruta, const std::string &txt)
{
  std::ofstream(ruta) << txt;
}

int main()
{
  const std::string tmp = "prueba_tmp.csv";
  std::string err;

  std::cout << "[1] catalogo valido (datos/cubos.csv)\n";
  std::vector<InfoCubo> cat;
  CHECK(cargarCatalogo("datos/cubos.csv", cat, err));
  CHECK(cat.size() == 5);
  CHECK(cat[0].color == "rojo" && cat[0].pesoReal == 300.0f && cat[0].q1 == -0.6);
  CHECK(cat[4].color == "morado" && cat[4].r == 0.40);

  std::cout << "[2] catalogos invalidos se rechazan con mensaje\n";
  std::vector<InfoCubo> x;
  CHECK(!cargarCatalogo("no_existe.csv", x, err) && err.find("no se pudo abrir") != std::string::npos);
  escribir(tmp, "1,rojo,-0.6,0.40\n");
  CHECK(!cargarCatalogo(tmp, x, err) && err.find("5 campos") != std::string::npos);
  escribir(tmp, "1,rojo,-0.6,0.40,300\n3,azul,0,0.4,60\n");
  CHECK(!cargarCatalogo(tmp, x, err) && err.find("consecutivos") != std::string::npos);
  escribir(tmp, "1,rojo,abc,0.40,300\n");
  CHECK(!cargarCatalogo(tmp, x, err) && err.find("invalido") != std::string::npos);
  escribir(tmp, "1,rojo,-0.6,0.40,300x\n");
  CHECK(!cargarCatalogo(tmp, x, err));
  escribir(tmp, "1,rojo,-0.6,-0.40,300\n");
  CHECK(!cargarCatalogo(tmp, x, err) && err.find("rango") != std::string::npos);
  escribir(tmp, "# solo comentarios\n\n");
  CHECK(!cargarCatalogo(tmp, x, err));
  CHECK(x.empty());   // un error no modifica la salida

  std::cout << "[3] guardar y cargar pesos (ida y vuelta)\n";
  std::vector<Cubo> lect = {{1, 301.2f, -0.6, 0.40}, {2, 59.1f, -0.3, 0.46}, {3, 449.5f, 0.0, 0.42},
                            {4, 120.7f, 0.3, 0.46}, {5, 199.9f, 0.6, 0.40}};
  CHECK(guardarPesos(tmp, lect, err));
  std::vector<Cubo> leidos;
  CHECK(cargarPesos(tmp, cat, leidos, err));
  CHECK(leidos.size() == 5);
  for (size_t i = 0; i < leidos.size(); ++i)
  {
    CHECK(leidos[i].id == lect[i].id);
    CHECK(std::fabs(leidos[i].peso - lect[i].peso) < 0.01f);
    CHECK(leidos[i].q1 == cat[i].q1 && leidos[i].r == cat[i].r);   // q1 y r vienen del catalogo
  }
  escribir(tmp, "id,peso_g\n9,100\n");
  CHECK(!cargarPesos(tmp, cat, leidos, err) && err.find("catalogo") != std::string::npos);
  escribir(tmp, "1,100\n1,200\n");
  CHECK(!cargarPesos(tmp, cat, leidos, err) && err.find("repetido") != std::string::npos);
  escribir(tmp, "1,-5\n");
  CHECK(!cargarPesos(tmp, cat, leidos, err));
  CHECK(!guardarPesos("carpeta_que_no_existe/p.csv", lect, err));

  std::cout << "[4] BST + Merge Sort + Busqueda Binaria con los pesos nominales\n";
  std::vector<Cubo> nominales;
  for (const InfoCubo &c : cat) nominales.push_back(Cubo{c.id, c.pesoReal, c.q1, c.r});
  BST<Cubo> arbol;
  for (const Cubo &c : nominales) arbol.add(c);
  CHECK(arbol.size() == 5);
  CHECK(arbol.height() == 4);   // 300 -> 60 -> 120 -> 200 (insercion en orden de id)
  CHECK(arbol.minimum()->id == 2 && arbol.maximum()->id == 3);

  Sorts<Cubo> sorts;
  std::vector<Cubo> orden = nominales;
  sorts.ordenaMerge(orden);
  std::vector<Cubo> inorden;
  arbol.inorderApply([&](const Cubo &c) { inorden.push_back(c); });
  CHECK(orden.size() == inorden.size());
  for (size_t i = 0; i < orden.size(); ++i) CHECK(orden[i].id == inorden[i].id);
  CHECK(orden[0].id == 2 && orden[1].id == 4 && orden[2].id == 5 && orden[3].id == 1 && orden[4].id == 3);

  CHECK(sorts.busqBinaria(orden, Cubo{0, 200.0f, 0, 0}) == 2);
  CHECK(sorts.busqBinaria(orden, Cubo{0, 202.0f, 0, 0}) == 2);   // dentro de la tolerancia de 5 g
  CHECK(sorts.busqBinaria(orden, Cubo{0, 250.0f, 0, 0}) == -1);
  CHECK(sorts.busqBinaria(std::vector<Cubo>(), Cubo{0, 1.0f, 0, 0}) == -1);
  CHECK(arbol.find(Cubo{0, 202.0f, 0, 0}) != nullptr && arbol.find(Cubo{0, 250.0f, 0, 0}) == nullptr);

  std::vector<Cubo> vacio;
  sorts.ordenaMerge(vacio);   // no debe fallar
  CHECK(vacio.empty());

  std::remove(tmp.c_str());
  std::cout << (fallos == 0 ? "TODAS LAS PRUEBAS PASARON\n" : "HAY FALLOS\n");
  return fallos == 0 ? 0 : 1;
}
