// cubo.h
// Dato que guarda el BST (Cubo), catalogo de cubos y LECTURA/ESCRITURA DE ARCHIVOS CSV.
// No depende de Gazebo, asi que se puede compilar y probar solo (ver pruebas/pruebas_datos.cc).
//
// Archivos:
//   datos/cubos.csv   catalogo:  id,color,q1_rad,r_m,peso_real_g     (entrada obligatoria)
//   datos/pesos.csv   pesos medidos: id,peso_g                       (se guarda/carga con el menu)
// Las lineas vacias y las que empiezan con '#' se ignoran; la linea de encabezado (empieza con "id")
// tambien.
#ifndef CUBO_H_
#define CUBO_H_

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

constexpr float TOL_G = 5.0f;  // tolerancia de la busqueda por peso (g)

// Una fila de datos/cubos.csv
struct InfoCubo
{
  int id;
  std::string color;
  double q1;       // angulo de la base donde esta el cubo (rad)
  double r;        // radio al que esta el cubo (m)
  float pesoReal;  // peso nominal (g), solo para mostrar el error de la medicion
};

// Dato que guarda el arbol
struct Cubo
{
  int id;        // 1..N
  float peso;    // gramos (medido con el sensor)
  double q1;     // angulo de la base donde esta el cubo
  double r;      // radio al que esta el cubo

  bool operator<(const Cubo &o) const { return peso < o.peso; }
  bool operator<=(const Cubo &o) const { return peso <= o.peso; }   // lo usa Merge Sort
  // Igualdad con tolerancia: el peso medido nunca es exacto
  bool operator==(const Cubo &o) const { return std::fabs(peso - o.peso) < TOL_G; }
};

// Formato "C3 450.0g". Se arma en un stringstream aparte para no alterar el formato de cout.
inline std::ostream &operator<<(std::ostream &os, const Cubo &c)
{
  std::ostringstream tmp;
  tmp << "C" << c.id << " " << std::fixed << std::setprecision(1) << c.peso << "g";
  return os << tmp.str();
}

// ---------------------------------------------------------------------------------------------
// Lectura de CSV
// ---------------------------------------------------------------------------------------------
namespace csv
{
inline std::string recortar(const std::string &s)
{
  size_t a = 0, b = s.size();
  while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
  while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) --b;
  return s.substr(a, b - a);
}

inline std::vector<std::string> separar(const std::string &linea)
{
  std::vector<std::string> campos;
  std::string campo;
  std::stringstream ss(linea);
  while (std::getline(ss, campo, ',')) campos.push_back(recortar(campo));
  if (!linea.empty() && linea.back() == ',') campos.push_back("");
  return campos;
}

// Convierte TODO el texto o falla (rechaza "12abc" y "")
inline bool aEntero(const std::string &s, int &out)
{
  try { size_t pos = 0; out = std::stoi(s, &pos); return pos == s.size(); }
  catch (...) { return false; }
}

inline bool aDouble(const std::string &s, double &out)
{
  try { size_t pos = 0; out = std::stod(s, &pos); return pos == s.size() && std::isfinite(out); }
  catch (...) { return false; }
}

// Lee un archivo y entrega solo las lineas de datos (con su numero de linea original)
inline bool leerLineas(const std::string &ruta, std::vector<std::pair<int, std::string>> &filas,
                       std::string &error)
{
  std::ifstream in(ruta);
  if (!in)
  {
    error = "no se pudo abrir '" + ruta + "'";
    return false;
  }
  std::string linea;
  int n = 0;
  while (std::getline(in, linea))
  {
    ++n;
    std::string t = recortar(linea);
    if (t.empty() || t[0] == '#') continue;
    if (t.size() >= 2 && (t[0] == 'i' || t[0] == 'I') && (t[1] == 'd' || t[1] == 'D')) continue;  // encabezado
    filas.push_back({n, t});
  }
  return true;
}
}  // namespace csv

// Carga datos/cubos.csv. Valida: 5 campos, id consecutivo desde 1 (el programa usa INFO[id-1]),
// numeros completos, radio > 0, peso > 0 y |q1| <= pi.
inline bool cargarCatalogo(const std::string &ruta, std::vector<InfoCubo> &out, std::string &error)
{
  std::vector<std::pair<int, std::string>> filas;
  if (!csv::leerLineas(ruta, filas, error)) return false;

  std::vector<InfoCubo> res;
  for (const auto &f : filas)
  {
    std::string donde = "linea " + std::to_string(f.first) + " de '" + ruta + "': ";
    std::vector<std::string> c = csv::separar(f.second);
    if (c.size() != 5)
    {
      error = donde + "se esperaban 5 campos (id,color,q1,r,peso) y hay " + std::to_string(c.size());
      return false;
    }
    InfoCubo info;
    double peso;
    if (!csv::aEntero(c[0], info.id) || !csv::aDouble(c[2], info.q1) || !csv::aDouble(c[3], info.r) ||
        !csv::aDouble(c[4], peso))
    {
      error = donde + "hay un campo numerico invalido";
      return false;
    }
    if (info.id != static_cast<int>(res.size()) + 1)
    {
      error = donde + "los id deben ser consecutivos desde 1 (se esperaba " +
              std::to_string(res.size() + 1) + ")";
      return false;
    }
    if (c[1].empty()) { error = donde + "el color esta vacio"; return false; }
    if (info.r <= 0.0 || peso <= 0.0 || std::fabs(info.q1) > 3.14)
    {
      error = donde + "valores fuera de rango (r > 0, peso > 0, |q1| <= 3.14)";
      return false;
    }
    info.color = c[1];
    info.pesoReal = static_cast<float>(peso);
    res.push_back(info);
  }
  if (res.empty()) { error = "'" + ruta + "' no contiene cubos"; return false; }
  out = res;
  return true;
}

// Guarda los pesos medidos: id,peso_g
inline bool guardarPesos(const std::string &ruta, const std::vector<Cubo> &v, std::string &error)
{
  std::ofstream out(ruta);
  if (!out)
  {
    error = "no se pudo escribir '" + ruta + "'";
    return false;
  }
  out << "# Pesos medidos con el sensor F/T (id,peso_g)\n"
      << "id,peso_g\n";
  for (const Cubo &c : v) out << c.id << "," << std::fixed << std::setprecision(2) << c.peso << "\n";
  out.flush();
  if (!out)
  {
    error = "fallo la escritura de '" + ruta + "'";
    return false;
  }
  return true;
}

// Carga pesos guardados. q1 y r se toman del catalogo (asi no pueden quedar inconsistentes).
// Rechaza ids que no esten en el catalogo, ids repetidos, y pesos no positivos.
inline bool cargarPesos(const std::string &ruta, const std::vector<InfoCubo> &catalogo,
                        std::vector<Cubo> &out, std::string &error)
{
  std::vector<std::pair<int, std::string>> filas;
  if (!csv::leerLineas(ruta, filas, error)) return false;

  std::vector<Cubo> res;
  std::set<int> vistos;
  for (const auto &f : filas)
  {
    std::string donde = "linea " + std::to_string(f.first) + " de '" + ruta + "': ";
    std::vector<std::string> c = csv::separar(f.second);
    if (c.size() != 2) { error = donde + "se esperaban 2 campos (id,peso)"; return false; }
    int id;
    double peso;
    if (!csv::aEntero(c[0], id) || !csv::aDouble(c[1], peso))
    {
      error = donde + "hay un campo numerico invalido";
      return false;
    }
    if (id < 1 || id > static_cast<int>(catalogo.size()))
    {
      error = donde + "el id " + std::to_string(id) + " no esta en el catalogo";
      return false;
    }
    if (!vistos.insert(id).second) { error = donde + "id repetido"; return false; }
    if (peso <= 0.0) { error = donde + "el peso debe ser positivo"; return false; }
    res.push_back(Cubo{id, static_cast<float>(peso), catalogo[id - 1].q1, catalogo[id - 1].r});
  }
  if (res.empty()) { error = "'" + ruta + "' no contiene pesos"; return false; }
  out = res;
  return true;
}

#endif /* CUBO_H_ */
