//Jose Arturo Perez Cruz A01713728
//Tarea 3 arbol de busqueda binaria
//
// Adaptado para el proyecto arm_bst. Se conserva el arbol original (add, visit, height,
// ancestors, whatlevelamI) y se agregan los metodos que necesita el brazo:
// find, findIf, minimum, maximum, range, inorderApply, printTree, size, clear.
//
// El tipo T debe tener:  operator<   (para ordenar)
//                        operator==  (para buscar; puede tener tolerancia)
//                        operator<<  (para visit / ancestors / printTree)

#ifndef BST_H_
#define BST_H_

#include <iostream>
#include <sstream>
#include <string>
#include <queue>

template <class T>
class BST;

template <class T>
class Node {
private:
    T value;
    Node<T> *left;
    Node<T> *right;

    Node(T val) : value(val), left(nullptr), right(nullptr) {}
    Node(T val, Node<T>* le, Node<T>* ri) : value(val), left(le), right(ri) {}

    friend class BST<T>;
};

template <class T>
class BST {
private:
    Node<T> *root;
    int count;

    int height(Node<T>* node) const;
    void deleteAll(Node<T>* node);

    void preorder(Node<T>* node, std::stringstream &ss) const;
    void inorder(Node<T>* node, std::stringstream &ss) const;
    void postorder(Node<T>* node, std::stringstream &ss) const;
    void levelByLevel(Node<T>* node, std::stringstream &ss) const;

    // ---- auxiliares de los metodos nuevos ----
    template <class F>
    void inorderApply(Node<T>* node, F &f) const;
    template <class F>
    void rangeApply(Node<T>* node, const T &lo, const T &hi, F &f) const;
    void printTree(Node<T>* node, int level) const;

    // El arbol maneja memoria con punteros: se prohibe copiarlo
    BST(const BST&);
    BST& operator=(const BST&);

public:
    BST();
    ~BST();

    bool empty() const;

    void add(T val);
    std::string visit() const;
    int height() const;
    std::string ancestors(T val) const;
    int whatlevelamI(T val) const;

    // ---- metodos nuevos ----
    int size() const;
    void clear();

    // Busca con == de T (tolerancia incluida). nullptr si no esta.
    const T* find(const T &val) const;

    // Primer elemento (en orden de menor a mayor) que cumple el predicado. nullptr si ninguno.
    template <class F>
    const T* findIf(F pred) const;

    const T* minimum() const;
    const T* maximum() const;

    // Llama f(x) para cada elemento de menor a mayor
    template <class F>
    void inorderApply(F f) const;

    // Llama f(x) para cada x con lo <= x <= hi, de menor a mayor (poda ramas)
    template <class F>
    void range(const T &lo, const T &hi, F f) const;

    // Dibuja el arbol de lado: raiz a la izquierda, mayores arriba
    void printTree() const;
};

template <class T>
BST<T>::BST() : root(nullptr), count(0) {}

template <class T>
BST<T>::~BST() {
    deleteAll(root);
    root = nullptr;
}

template <class T>
void BST<T>::deleteAll(Node<T>* node) {
    if (node == nullptr) {
        return;
    }
    deleteAll(node->left);
    deleteAll(node->right);
    delete node;
}

template <class T>
bool BST<T>::empty() const {
    return root == nullptr;
}

template <class T>
void BST<T>::add(T val) {
    Node<T> *newNode = new Node<T>(val);
    count++;

    if (root == nullptr) {
        root = newNode;
        return;
    }

    Node<T> *current = root;
    Node<T> *parent = nullptr;

    while (current != nullptr) {
        parent = current;
        if (val < current->value) {
            current = current->left;
        } else {
            current = current->right;
        }
    }

    if (val < parent->value) {
        parent->left = newNode;
    } else {
        parent->right = newNode;
    }
}

template <class T>
void BST<T>::preorder(Node<T>* node, std::stringstream &ss) const {
    if (node == nullptr) {
        return;
    }
    if (ss.tellp() > 1) {
        ss << " ";
    }
    ss << node->value;
    preorder(node->left, ss);
    preorder(node->right, ss);
}

template <class T>
void BST<T>::inorder(Node<T>* node, std::stringstream &ss) const {
    if (node == nullptr) {
        return;
    }
    inorder(node->left, ss);
    if (ss.tellp() > 1) {
        ss << " ";
    }
    ss << node->value;
    inorder(node->right, ss);
}

template <class T>
void BST<T>::postorder(Node<T>* node, std::stringstream &ss) const {
    if (node == nullptr) {
        return;
    }
    postorder(node->left, ss);
    postorder(node->right, ss);
    if (ss.tellp() > 1) {
        ss << " ";
    }
    ss << node->value;
}

template <class T>
void BST<T>::levelByLevel(Node<T>* node, std::stringstream &ss) const {
    if (node == nullptr) {
        return;
    }
    std::queue<Node<T>*> q;
    q.push(node);

    while (!q.empty()) {
        Node<T> *curr = q.front();
        q.pop();

        if (ss.tellp() > 1) {
            ss << " ";
        }
        ss << curr->value;

        if (curr->left != nullptr) {
            q.push(curr->left);
        }
        if (curr->right != nullptr) {
            q.push(curr->right);
        }
    }
}

template <class T>
std::string BST<T>::visit() const {
    std::stringstream pre, in, post, level;
    pre << "[";
    in << "[";
    post << "[";
    level << "[";

    preorder(root, pre);
    inorder(root, in);
    postorder(root, post);
    levelByLevel(root, level);

    pre << "]";
    in << "]";
    post << "]";
    level << "]";

    std::stringstream ss;
    ss << pre.str() << "\n"
       << in.str() << "\n"
       << post.str() << "\n"
       << level.str();
    return ss.str();
}

template <class T>
int BST<T>::height(Node<T>* node) const {
    if (node == nullptr) {
        return 0;
    }
    int lh = height(node->left);
    int rh = height(node->right);
    return 1 + (lh > rh ? lh : rh);
}

template <class T>
int BST<T>::height() const {
    return height(root);
}

// Con tolerancia en == el arbol sigue siendo consistente: si dos valores NO son iguales
// (difieren mas que la tolerancia), entonces < decide bien hacia que lado ir.
template <class T>
std::string BST<T>::ancestors(T val) const {
    std::stringstream ss;
    ss << "[";

    Node<T> *current = root;
    while (current != nullptr && !(current->value == val)) {
        current = (val < current->value) ? current->left : current->right;
    }

    if (current != nullptr) {
        current = root;
        bool first = true;
        while (!(current->value == val)) {
            if (!first) {
                ss << " ";
            }
            ss << current->value;
            first = false;
            current = (val < current->value) ? current->left : current->right;
        }
    }

    ss << "]";
    return ss.str();
}

template <class T>
int BST<T>::whatlevelamI(T val) const {
    Node<T> *current = root;
    int level = 1;

    while (current != nullptr) {
        if (val == current->value) {
            return level;
        }
        current = (val < current->value) ? current->left : current->right;
        level++;
    }
    return -1;
}

// ===================== metodos nuevos =====================

template <class T>
int BST<T>::size() const {
    return count;
}

template <class T>
void BST<T>::clear() {
    deleteAll(root);
    root = nullptr;
    count = 0;
}

template <class T>
const T* BST<T>::find(const T &val) const {
    Node<T> *current = root;
    while (current != nullptr) {
        if (val == current->value) {
            return &current->value;
        }
        current = (val < current->value) ? current->left : current->right;
    }
    return nullptr;
}

template <class T>
template <class F>
void BST<T>::inorderApply(Node<T>* node, F &f) const {
    if (node == nullptr) {
        return;
    }
    inorderApply(node->left, f);
    f(node->value);
    inorderApply(node->right, f);
}

template <class T>
template <class F>
void BST<T>::inorderApply(F f) const {
    inorderApply(root, f);
}

template <class T>
template <class F>
const T* BST<T>::findIf(F pred) const {
    const T *res = nullptr;
    inorderApply([&](const T &x) {
        if (res == nullptr && pred(x)) {
            res = &x;
        }
    });
    return res;
}

template <class T>
const T* BST<T>::minimum() const {
    Node<T> *current = root;
    if (current == nullptr) {
        return nullptr;
    }
    while (current->left != nullptr) {
        current = current->left;
    }
    return &current->value;
}

template <class T>
const T* BST<T>::maximum() const {
    Node<T> *current = root;
    if (current == nullptr) {
        return nullptr;
    }
    while (current->right != nullptr) {
        current = current->right;
    }
    return &current->value;
}

template <class T>
template <class F>
void BST<T>::rangeApply(Node<T>* node, const T &lo, const T &hi, F &f) const {
    if (node == nullptr) {
        return;
    }
    bool menorQueLo = node->value < lo;  // node < lo: todo el subarbol izquierdo tambien
    bool mayorQueHi = hi < node->value;  // node > hi: todo el subarbol derecho tambien
    if (!menorQueLo) {
        rangeApply(node->left, lo, hi, f);
    }
    if (!menorQueLo && !mayorQueHi) {
        f(node->value);
    }
    if (!mayorQueHi) {
        rangeApply(node->right, lo, hi, f);
    }
}

template <class T>
template <class F>
void BST<T>::range(const T &lo, const T &hi, F f) const {
    rangeApply(root, lo, hi, f);
}

template <class T>
void BST<T>::printTree(Node<T>* node, int level) const {
    if (node == nullptr) {
        return;
    }
    printTree(node->right, level + 1);
    std::cout << std::string(level * 6, ' ') << (level > 0 ? "|-- " : "") << node->value << "\n";
    printTree(node->left, level + 1);
}

template <class T>
void BST<T>::printTree() const {
    if (root == nullptr) {
        std::cout << "  (arbol vacio)\n";
        return;
    }
    printTree(root, 0);
}

#endif /* BST_H_ */
