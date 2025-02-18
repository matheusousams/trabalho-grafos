from biblioteca import Digrafo
from biblioteca import Grafo


if __name__ == "__main__":

    digrafo = Digrafo()
    with open("USA-road-d.NY.gr", "r") as arquivo:
        for _ in range(7):
            next(arquivo)
        
        for linha in arquivo:
            partes=linha.strip().split()
            if partes[0]=="a":
                _,u,v,w = partes
                digrafo.inserirArestas(int(u),int(v),int(w)) #chamada da função inserirArestas convertendo os valores string para int


    # a) O valor de G.mind
    print(f"O valor do menor grau: {digrafo.menorGrau()}")
    print("----------------------------------------------------------------")
    # b) O valor de G.maxd
    print(f"O valor do maior grau: {digrafo.maiorGrau()}")
    print("----------------------------------------------------------------")
    # c) Um caminho com uma qtde. de arestas maior ou igual a 10 (apresentar a sequência dos vértices).
    print(f"Caminho com pelo menos 10 arestas: {digrafo.caminho(1)}")
    print("----------------------------------------------------------------")
    # d) Um ciclo com uma qtde. de arestas maior ou igual a 5 (apresentar a sequência dos vértices).
    print(f"ciclo com uma qtde. de arestas maior ou igual a 5: {digrafo.ciclo(1)}")
    print("----------------------------------------------------------------")
    # e) O vértice mais distante (considerando os pesos das arestas) do vértice 129, e o valor da distância entre eles.
    print(digrafo.verticeMaisDistante(129))