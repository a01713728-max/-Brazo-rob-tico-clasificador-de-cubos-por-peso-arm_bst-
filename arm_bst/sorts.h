// sorts.h
// Algoritmos de ordenamiento y busqueda del avance 1 (Merge Sort y Busqueda Binaria).
// En el avance 2 se usan en control_bst.cc para ordenar las lecturas de peso y validar
// el recorrido inorden del BST (opcion 10 del menu).
//
// El tipo T debe tener operator<, operator<= y operator==.
#ifndef SORTS_H
#define SORTS_H

#include <iostream>
#include <vector>

template <class T>
class Sorts
{
private:
    void swap(std::vector<T>&, int, int);
    void copyArray(std::vector<T>&, std::vector<T>&, int, int);
    void mergeArray(std::vector<T>&, std::vector<T>&, int, int, int);
    void mergeSplit(std::vector<T>&, std::vector<T>&, int, int);

public:
    void ordenaMerge(std::vector<T>&);
    int busqBinaria(const std::vector<T>&, T);
};

template <class T>
void Sorts<T>::swap(std::vector<T>& v, int i, int j)
{
    T aux = v[i];
    v[i] = v[j];
    v[j] = aux;
}

template <class T>
void Sorts<T>::copyArray(std::vector<T>& v, std::vector<T>& B, int low, int high)
{
    for (int i = low; i <= high; i++)
    {
        v[i] = B[i];
    }
}

template <class T>
void Sorts<T>::mergeArray(std::vector<T>& v, std::vector<T>& B, int low, int mid, int high)
{
    int i_izquierda = low;
    int i_derecha = mid + 1;
    int i_ordenado = low;

    while (i_izquierda <= mid && i_derecha <= high)
    {
        if (v[i_izquierda] <= v[i_derecha])
        {
            B[i_ordenado] = v[i_izquierda];
            i_izquierda++;
        }
        else
        {
            B[i_ordenado] = v[i_derecha];
            i_derecha++;
        }
        i_ordenado++;
    }

    while (i_izquierda <= mid)
    {
        B[i_ordenado] = v[i_izquierda];
        i_izquierda++;
        i_ordenado++;
    }
    while (i_derecha <= high)
    {
        B[i_ordenado] = v[i_derecha];
        i_derecha++;
        i_ordenado++;
    }
    copyArray(v, B, low, high);
}

template <class T>
void Sorts<T>::mergeSplit(std::vector<T>& v, std::vector<T>& B, int low, int high)
{
    if (low < high)
    {
        int mitad = low + (high - low) / 2;
        mergeSplit(v, B, low, mitad);
        mergeSplit(v, B, mitad + 1, high);
        mergeArray(v, B, low, mitad, high);
    }
}

template <class T>
void Sorts<T>::ordenaMerge(std::vector<T>& v)
{
    if (v.empty()) return;
    std::vector<T> tmp(v.size());
    mergeSplit(v, tmp, 0, static_cast<int>(v.size()) - 1);
}

template <class T>
int Sorts<T>::busqBinaria(const std::vector<T>& v, T val)
{
    int low = 0;
    int high = static_cast<int>(v.size()) - 1;

    while (low <= high)
    {
        int mitad = low + (high - low) / 2;
        if (v[mitad] == val)
        {
            return mitad;
        }
        if (v[mitad] < val)
        {
            low = mitad + 1;
        }
        else
        {
            high = mitad - 1;
        }
    }

    return -1;
}

#endif
