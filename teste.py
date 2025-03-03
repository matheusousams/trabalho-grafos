from biblioteca import Digrafo

d = Digrafo()

d.inserirArestas(1, 2, 803)
d.inserirArestas(2, 1, 803)
d.inserirArestas(3, 4, 158)
d.inserirArestas(4, 3, 158)
d.inserirArestas(5, 6, 774)
d.inserirArestas(6, 5, 774)
d.inserirArestas(7, 8, 1531)
d.inserirArestas(8, 7, 1531)
d.inserirArestas(9, 10, 1673)
d.inserirArestas(10, 9, 1673)
d.inserirArestas(9, 11, 1400)
d.inserirArestas(11, 9, 1400)
d.inserirArestas(1, 12, 842)
d.inserirArestas(12, 1, 842)
d.inserirArestas(2, 13, 591)
d.inserirArestas(13, 2, 591)
d.inserirArestas(14, 15, 1371)
d.inserirArestas(15, 14, 1371)
d.inserirArestas(16, 17, 1659)
d.inserirArestas(17, 16, 1659)
d.inserirArestas(18, 19, 1012)
d.inserirArestas(19, 18, 1012)
d.inserirArestas(20, 21, 1226)
d.inserirArestas(21, 20, 1226)
d.inserirArestas(20, 22, 1265)
d.inserirArestas(22, 20, 1265)
d.inserirArestas(23, 24, 2707)
d.inserirArestas(24, 23, 2707)
d.inserirArestas(25, 26, 520)
d.inserirArestas(26, 25, 520)
d.inserirArestas(27, 28, 783)
d.inserirArestas(28, 27, 783)
d.inserirArestas(29, 30, 518)
d.inserirArestas(30, 29, 518)
d.inserirArestas(31, 32, 3412)
d.inserirArestas(32, 31, 3412)
d.inserirArestas(33, 34, 1994)
d.inserirArestas(34, 33, 1994)
d.inserirArestas(33, 35, 337)
d.inserirArestas(35, 33, 337)
d.inserirArestas(36, 37, 363)
d.inserirArestas(37, 36, 363)
d.inserirArestas(38, 39, 1018)
d.inserirArestas(39, 38, 1018)


print(f"busca em profundidade: {d.buscaEmProfundidade(1)}")
# a) O valor de G.mind
print(f"O valor de G.mind: {d.menorGrau()}")
# b) O valor de G.maxd
print(f"O valor de G.maxd: {d.maiorGrau()}")
# c) Um caminho com uma qtde. de arestas maior ou igual a 10 (apresentar a sequência dos vértices).
print(f"Caminho com pelo menos 10 arestas: {d.caminho(34)}")
# d) Um ciclo com uma qtde. de arestas maior ou igual a 5 (apresentar a sequência dos vértices).
print(f"ciclo com uma qtde. de arestas maior ou igual a 5: {d.ciclo(1)}")


print(f"bf: {d.bellmanFord(1)}")