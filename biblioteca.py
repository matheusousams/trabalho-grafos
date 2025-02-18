from collections import deque
import heapq


class Grafo:

    def __init__(self):
        
        #atributo para lista de adjacência
        self.listaDeAdjacencia ={

        }
        

    #função para inserir arestas em grafos não direcionados
    def inserirArestas(self, u, v, w=1):
        
        #inserção de arestas de forma bidirecional,por se tratar de grafos não-direcionais
        if u not in self.listaDeAdjacencia:
            self.listaDeAdjacencia[u]=[

            ]
        if v not in self.listaDeAdjacencia:
            self.listaDeAdjacencia[v]=[

            ]

        self.listaDeAdjacencia[u].append((v,w))
        self.listaDeAdjacencia[v].append((u,w))


    #retorna o número de vértices
    def numeroDeVertices(self):
        #cacula o tamanho da lista de adjacencia para descobrir o número de vértices
        return len(self.listaDeAdjacencia)

    
    #função que retorna o numero de arestas
    def numeroDeArestas(self):
        # o retorno é dividido por dois por se tratar de grafos bidirecionais
        return sum(len(v) for v in self.listaDeAdjacencia.values())//2
    

    #retorna vertices adjacentes
    def verticesAdjacentes(self, v):

        #retorna os uma lista com vértices que possuem conectividade com o vértice 'v' 
        if v in self.listaDeAdjacencia:
            return [
                    u for u, _ in self.listaDeAdjacencia[v]
                   ]
        else:
            return [

            ]


    #retorn grau do vértice
    def grauDoVertice(self,v):
        # calcula o tamanho da lista que representa a vizinhança de determinado
        #  vértice 'v' para descobrir qual o grau dele
        return len(self.verticesAdjacentes(v))
    

    #retorna peso da aresta
    def pesoDaAresta(self,u,v):

        #percorre uma determinada aresta e retorna seu peso
        for vertice, peso in self.listaDeAdjacencia.get(u, []):
            if vertice ==v:
                return peso
        return None
    

    #retorna o menor grau presente no grafo
    def menorGrau(self):

        #percorre todos os vértices para descobrir o grau de cada um e retorna o menor
        if [self.grauDoVertice(v) for v in self.listaDeAdjacencia.keys()]:
            return min([self.grauDoVertice(v) for v in self.listaDeAdjacencia.keys()])
        else:
            return 0
    

    #retorna o maior grau presente no grafo
    def maiorGrau(self):
        #percorre todos os vértices para descobrir o grau de cada um e retorna o menor
        if [self.grauDoVertice(v) for v in self.listaDeAdjacencia.keys()]:
            return max([self.grauDoVertice(v) for v in self.listaDeAdjacencia.keys()])
        else:
            return 0
        

    def buscaEmLargura(self,v):

        d = {key: float('inf') for key in self.listaDeAdjacencia} # a distância inicializa como infinito
        pi = {key: -1 for key in self.listaDeAdjacencia} # o predecessor é inicializado sem predecessor (-1)
        
        #a distância do vértice de origem para si mesmo é inicializado com 0
        d[v]=0
        #fila para inicializar a busca
        fila = deque([v])

        while fila:
            #o primeiro elemento da fila é removido
            atual=fila.popleft()
            for adjacente, _ in self.listaDeAdjacencia.get(atual,[]):
                # condicional que verifica se o vértice não foi visitado, caso não tenha sido visitado,
                # o vértice é adicionado na fila, a distância mínima é atualizada e o predecessor do
                # vértice é definido
                if d[adjacente]==float('inf'):
                    fila.append(adjacente)
                    d[adjacente]=d[atual]+1
                    pi[adjacente]=atual
        return d, pi


    def caminho(self,v):
        
        #atributos que armazenam a distancia entre cada vértice e 'v' e 
        # o vértice predecessor no caminho de 'v' até cada vértice chamando
        # o método de busca em largura
        d,pi=self.buscaEmLargura(v)
        #atributo que armazena o caminho de vértices
        caminhoDeVertices=[]

        #percorre todos os vértices da lista de distÂncia
        for vertice in d:
            #verifica se a distancia do primeiro vértice da lista é menor ou que
            # 10, se for, o atributo 'verticeAtual' recebe o valor do vértice
            if d[vertice]>=10:
                verticeAtual = vertice
                #o atributo 'verticeAtual' até chegar na raíz, adiciona 'verticeAtual' 
                # ao caminho e move para o vértice predecessor
                while verticeAtual!=-1:
                    caminhoDeVertices.append(verticeAtual)
                    verticeAtual=pi[verticeAtual]

                 #inverte a lista e retorna o caminho encontrado   
                caminhoDeVertices.reverse()
                return caminhoDeVertices
            
        #caso nenhum caminho seja encontrado, retorna a mensagem abaixo
        return "nenhum caminho encontrado"
    

    def buscaEmProfundidade(self,v):

        #atributo que representa os vértices visitados
        verticeVisitado=set()
        self.tempo=0 #variavel criada pra fazer o controle de tempo
        #atributos para listas que armazenam o vértice predecessor na árvore de busca
        pi={}
        #o tempo de início da visita a cada vértice
        ini={}
        # e a lista que indica o tempo de término da visita a cada vértice
        fim = {}

        #função criada para que a busca seja feita recursivamente
        def visitando(u):
            #marca o tempo que o vértice é descoberto e o adiciona no conjunto de
            #vértices visitados
            self.tempo+=1
            ini[u]=self.tempo
            verticeVisitado.add(u)
            #percorre os vértices adjacentes do vértice atual
            for adjacente, _ in self.listaDeAdjacencia.get(u,[]):
                if adjacente not in verticeVisitado:
                    #define o predecessor
                    pi[adjacente]=u
                    visitando(adjacente)
            # marca o tempo de finalização do vértice
            self.tempo+=1
            fim[u]=self.tempo
        
        #inicialização da função a partir do vértice fornecido pelo usuário
        visitando(v)
        return pi, ini, fim
    
    
    def ciclo(self, v, arestas=5):

        # atributo que serve para identificar vértices que já foram visitados
        verticeVisitado=set()
        #lista para armazenar o caminho
        caminhoDoVertice=[]

        def cicloDFS(u,origem):
            #condição criada ára parar quando o caminho passar 5 vértices
            if len(caminhoDoVertice) > arestas:
                return True
            verticeVisitado.add(u)
            caminhoDoVertice.append(u)

            for adjacente, _ in self.listaDeAdjacencia.get(u,[]):
                #if adjacente==origem and len(caminho)>=arestas:
                #    return True
                if adjacente==origem:
                    #verifica se o ciclo tem pelo menos 'arestas' arestas
                    if len(caminhoDoVertice)>=arestas:
                        #fecha o ciclo adicionado a origem
                        caminhoDoVertice.append(adjacente)
                        return True
                if adjacente not in verticeVisitado:
                    if cicloDFS(adjacente,origem):
                        return True
            
            #remove o vértice do caminho e faz a desmarcação como visitado
            caminhoDoVertice.pop()
            verticeVisitado.remove(u)
            return False
        
        # faz o inicio da busca a partir de 'v'  e retorna o ciclo caso 
        # seja encotrado
        if cicloDFS(v,v):
            return caminhoDoVertice
        return None


    def bellmanFord(self, v):

        #atributo para incicializar a lista de distÂncias 'd' com infinito
        d = {key: float('inf') for key in self.listaDeAdjacencia}
        #atributo para inicializar a lista de predecessores 'pi' com -1
        pi = {key: None for key in self.listaDeAdjacencia}
        #atributo que serve para definir a distancia do vertice 'v' como 0
        d[v] = 0
        
        #execução do processo de relaxamento de arestas
        for _ in range(len(self.listaDeAdjacencia) - 1):
            for u in self.listaDeAdjacencia:
                for adjacente, peso in self.listaDeAdjacencia[u]:
                    if d[u] != float('inf') and d[u] + peso < d[adjacente]:
                        d[adjacente] = d[u] + peso
                        pi[adjacente] = u
        
        return d, pi


    def dijkstra(self, v):

        #atributo para incicializar a lista de distÂncias 'd' com infinito
        d = {key: float('inf') for key in self.listaDeAdjacencia}
        #atributo para inicializar a lista de predecessores 'pi' com -1
        pi = {key: None for key in self.listaDeAdjacencia}
        #atributo que serve para definir a distancia do vertice 'v' como 0
        d[v] = 0
        #atribuito que serve para criar um min-heap (fila de prioridade) para
        #fazer o armazenamento dos vértices que serão processados
        fila = [
                (0, v)
            ]
        
        #execuçao de estrutura de repetição enquanto houver elementos
        # na fila de prioridade
        while fila:
            distancia,u= heapq.heappop(fila)
            if distancia>d[u]:
                continue
            for adjacente, peso in self.listaDeAdjacencia.get(u,[]):
                if d[u] + peso < d[adjacente]:
                    d[adjacente] = d[u]+peso
                    pi[adjacente] = u
                    heapq.heappush(fila,(d[adjacente], adjacente))
        
        return d, pi
    

    def verticeMaisDistante(self,verticeDeOrigem):
        
        #verifica se há pesos negativos
        pesosNegativos = False

        for u in self.listaDeAdjacencia:
            for v,w in self.listaDeAdjacencia[u]:
                if w<0:
                    pesosNegativos=True
                    break #otimiza: para se encontrar um peso negativo

        if pesosNegativos:
            #caso sejam encontrados pesos negativos, o algoritimo utilizado será bellman ford
            d, pi= self.bellmanFord(verticeDeOrigem)
        else:
            #caso NÂO sejam encontrados pesos negativos, o algoritimo utilizado será dijkstra
            d, pi= self.dijkstra(verticeDeOrigem)
        
        #encontra o vértice mais distante e sua distância
        verticeMaisDistante=max(d, key=d.get)
        d = d[verticeMaisDistante]

        return f"O vértice mais distante de {verticeDeOrigem} é {verticeMaisDistante} \n A distância entre o vértice {verticeDeOrigem} e o vértice {verticeMaisDistante} é {d}"

        


class Digrafo(Grafo):
    def __init__(self):
        super().__init__()

    #método que adiciona arestas direcionadas por lista de adjacencia
    def inserirArestas(self,u,v,w=1):
        
        if u not in self.listaDeAdjacencia:
            self.listaDeAdjacencia[u]=[

            ]
        if v not in self.listaDeAdjacencia:
            self.listaDeAdjacencia[v]=[

            ]
        self.listaDeAdjacencia[u].append((v,w))

    def numeroDeArestas(self):

        #este retorno não dividirá por dois porque se trata de um grafo direcionado
        return sum(len(v) for v in self.listaDeAdjacencia.values())