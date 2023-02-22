from collections import deque
from random import randint
from time import time
import random

#classe para inicializarmos o grafo
class Grafo:
  def __init__(self, V, Adj, num_vertex):
    self.V = V
    self.Adj = Adj
    self.num_vertex = num_vertex

#classe para inicializarmos o vértice
class Vertice:
  def __init__(self, indice, d, pai, cor):
    self.indice = indice
    self.d = d
    self.pai = pai
    self.cor = cor
    self.rank = 0
    self.chave = 0

class GrafoKruskal:
  def __init__(self, V, Adj, num_vertex, aresta):
    self.V = V
    self.Adj = Adj
    self.num_vertex = num_vertex
    self.aresta = aresta

# função que enfileira um vértice v na fila criada em tempo O(1) 
def enqueue(Q, v):
    Q.append(v)

# função que desenfileira um vértice v da fila também em tempo O(1) 
def dequeue(Q):
    v = Q[0]
    Q.popleft() #utilizamos popleft pois precisamos manter o algoritmo em tempo 0(1), e a função pop() tem tempo O(n)
    return v

#função para encontrarmos o vértice do grafo que possui o maior valor do atributo .d, para isso utilizamos o algoritmo do BFS com pequenas modificações
#bfs implementado com base no algoritmo visto em aula
def bfsMax(G, s):
    s.d = 0 
    s.cor = 'cinza'
    s.pai = None

    maxValue = s #poderia ser inicializada com um vértice aleatório, então escolhemos o primeiro

    Q = deque([])
    enqueue(Q, s)

    while (len(Q) != 0):
        u = dequeue(Q)
        for v in G.Adj[u.indice]:
            # G é um grafo, V é uma lista do tipo Vertex, v tem as propriedades .indice, .d, .pai, .cor
            if G.V[v].cor == 'branco': 
                G.V[v].cor = 'cinza' 
                G.V[v].d = u.d + 1
                G.V[v].pai = u
                enqueue(Q, G.V[v])
                if G.V[v].d > maxValue.d:
                    maxValue = G.V[v]

        u.cor = 'preto'
    return maxValue

# função para verificar se um grafo é uma árvore, utilizando DFS
# retornando true caso sim, e false caso não seja.
def verifica_arvore(G):
    global tempo
    tempo = 0
    
    #DFSVisit retorna True se o grafo não possui ciclos, falso caso possua
    x = DFSVisit(G, G.V[0])

    #a variável tempo será responsável por verificar se o grafo é conexo
    #como temos o tempo de chegada e saída do vértice, ao dividirmos tempo/2 devemos obter a quantidade de vértices do grafo
    #caso tempo/2 e a quantidade de vértices sejam diferentes, temos que o grafo não é conexo
    if x and tempo/2 == G.num_vertex:
        return True
    else:
        return False
        
#função que executa o DFSVisit, semelhante à implementação vista em aula
#retorna falso caso o grafo não seja uma árvore, e true caso consiga completar sua execução por todos os vértices do grafo
# tempo = 0
def DFSVisit(G, u):
    global tempo

    u.cor = "cinza"
    tempo = tempo + 1
    u.d = tempo

    for v in G.Adj[u.indice]:
        if G.V[v].cor == "branco":
            # se no meio da chamada DFSVisit retornar que o grafo não é uma árvore
            if not DFSVisit(G, G.V[v]):
                return False
        elif G.V[v].cor == 'preto':
            return False

    u.cor = "preto"
    tempo += 1
    u.f = tempo

    return True

# função que calcula o diâmetro de uma árvore T, para isso calculamos o comprimento do maior caminho em T, retornando este valor ao final da execução
def diameter(T):
    #setamos o valor do vértice para branco
    for i in range(T.num_vertex):
        T.V[i].d = None
        T.V[i].pai = None
        T.V[i].cor = 'branco'
    s = T.V[0] # s = vértice qualquer de T
    a = bfsMax(T, s)
    #devemos resetar os valores dos vértices, pois até o momento o .d de todos os vértices possuem a distância em relação à s
    for i in range(T.num_vertex):
        T.V[i].d = None
        T.V[i].pai = None
        T.V[i].cor = 'branco'

    b = bfsMax(T, a)

    return b.d

#função que executa o random_tree_random_walk para um número n de vértices
#é realizada uma verificação se o grafo obtido é uma árvore
#caso o grafo seja uma árvore, o mesmo é retornado. Caso contrário, a função retorna None
def random_tree_random_walk(n):
    #inicializa o grafo com n vértices; inicializa uma lista de adjacência para cada execução
    G = Grafo([Vertice(i, None, None, 'branco') for i in range(n)], [[] for i in range(n)] , n) 

    u = G.V[0] # s = vértice qualquer de V

    arestas = 0

    #lista para todos os vértices que forem visitados
    visitados = [False for i in range(n)]

    #marca que o primeiro vértice foi visitado
    visitados[u.indice] = True

    #como toda árvore tem n-1 arestas, sendo n o número de vértices.
    while arestas < n-1:
        v = G.V[randint(0, n-1)] # gera um vértice aleatório

        # se o vértice v não tiver sido visitado
        #adicionamos a aresta (u,v) na lista de adjacências
        #aumentamos a quantidade de arestas
        #ao final, marca que o vértice v foi visitado
        if visitados[v.indice] == False:
            G.Adj[u.indice].append(v.indice) #adiciona v na lista de adjacência de u
            G.Adj[v.indice].append(u.indice) #adiciona u na lista de adjacência de v

            arestas += 1
            visitados[v.indice] = True
        u = v
        
    if verifica_arvore(G) == True:
        return G
    else:
        return None

#função utilizada para criar um grafo completo
#será utilizada no random_tree_kruskal e no random_tree_prim
def grafoCompleto(n):
    G = GrafoKruskal([Vertice(i, None, None, 'branco') for i in range(n)], [[] for i in range(n)] , n, []) 

    for x in range(n):
        for y in range(x+1, n):
            G.Adj[x].append(G.V[y])
            G.Adj[y].append(G.V[x])      
            G.aresta.append([x, y, random.random()])
    return G

#cria um grafo completo utilizando matriz de adjacências
#necessário para a execução do random_tree_prim
def grafoCompletoPrim(n):
    G = Grafo([Vertice(i, None, None, 'branco') for i in range(n)], [[float('inf') for j in range(n)] for i in range(n)] , n) 

    for x in range(n):
        for y in range(x+1, n):
            G.Adj[x][y] = random.random()
            G.Adj[y][x] = G.Adj[x][y]
    return G
    
##funções auxiliares para a execução do mst_kruskal
#todas as funções foram implementadas seguindo o pseudo-código apresentado no livro Introduction to Algorithms - Thomas H. Cormen

def make_set(v):
    v.pai = v
    v.rank = 0

#une os conjuntos que contém u e v
def uniao(u, v):
    link(find_set(u), find_set(v))

#liga os vértices u e v com base no rank
def link(u, v):
    if u.rank > v.rank:
        v.pai = u
    else:
        u.pai = v
        if u.rank == v.rank:
            v.rank = v.rank + 1

def find_set(u):
    if u != u.pai:
        u.pai = find_set(u.pai)
    return u.pai

#algoritmos para o método de kruskal
#algoritmos baseados no pseudo-código visto em aula

#ordena as arestas com base no peso
#verifica se os vértices que compoem a aresta estão no mesmo componente
#caso não estejam, adiciona a aresta na solução
#ao final, retorna a lista de arestas que compõem o mst_kruskal
def mst_kruskal(G):
    A = []
    
    for i in range(G.num_vertex):
        make_set(G.V[i])
    #ordena as arestas com base no peso  
    G.aresta.sort(key=lambda x :x[2])

    x = 0 
    y = 0

    while x < G.num_vertex - 1:
        (u, v, peso) = G.aresta[y]
        y += 1
        if find_set(G.V[u]) != find_set(G.V[v]):
            #adiciona a aresta (u,v) na solução
            A.append([u, v, peso])
            #liga os vértices u e v em suas respectivas listas de adjacência
            uniao(G.V[u],G.V[v])

            x += 1
    return A

#gera um grafo com os vértices retornados de mst_kruskal(), sendo esse uma árvore geradora mínima
def random_tree_kruskal(n):
    G = grafoCompleto(n)

    x = mst_kruskal(G)
    
    lista = [[]for i in range(n)] 

    for i in x:
        lista[i[0]].append(i[1])
        lista[i[1]].append(i[0])

    grafo2 = GrafoKruskal([Vertice(i, None, None, 'branco') for i in range(n)], lista, n, x)

    if verifica_arvore(grafo2) == True:
        return grafo2
    else:
        return None

#algoritmos para o método de prim
#olha todas as arestas dos vértices já visitados e escolhe a de menor peso, desde que o vértice de destino seja um vértice que não tenha sido visitado
#executa até que todos os vértices tenham sido visitados, gerando uma árvore geradora mínima

#função que extrai o vértice de menor peso da fila
def extract_min(Q):
    min = Q[0]
    for i in Q:
        if min.chave > i.chave:
            min = i
    Q.remove(min)
    return min

#propósito para funções mst_prim() e random_tree_prim()
#algoritmo para gerar uma árvore geradora mínima utilizando o método de Prim, adicionando um vértice por vez na solução
#a escolha do vértice será pelo vértice de menor peso possível
def mst_prim(G, s):
    for u in G.V:
        u.chave = float('inf')
        u.pai = None
    s.chave = 0

    Q = []

    #coloca os vértices de G na fila
    for i in G.V:
        Q.append(i)

    while (len(Q) != 0):
        u = extract_min(Q)

        for v in Q:         
            if G.V[v.indice].chave > G.Adj[u.indice][v.indice]:
                G.V[v.indice].chave = G.Adj[u.indice][v.indice]
                G.V[v.indice].pai = u

    #cria a lista de adjacências
    saida = [[] for i in range(len(G.V))]
 
    #coloca na lista de adjacências
    for u in G.V:
        #não podemos pegar o primeiro vértice, pois não tem pai
        if u.pai != None:
            saida[u.pai.indice].append(u.indice)
            saida[u.indice].append(u.pai.indice)
    
    return saida

#gera um grafo com os vértices retornados de mst_prim(), sendo esse uma árvore geradora mínima
def random_tree_prim(n):
    G = grafoCompletoPrim(n);

    lista = mst_prim(G, G.V[0]);

    grafo2 = Grafo([Vertice(i, None, None, 'branco') for i in range(n)], lista, n)

    if verifica_arvore(grafo2) == True:
        return grafo2
    else:
        return None

#testes automatizados para a função diameter
def assert_diameter():    
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[3], [2], [3, 1], [0, 4, 2], [3]], 5)) == 3
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[4, 2], [4], [0, 3], [2], [0, 1]], 5)) == 4
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[2], [4], [0, 3, 4], [2], [2, 1]], 5)) == 3
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[1, 4], [0], [4, 3], [2], [0, 2]], 5)) == 4
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[4, 3, 2], [4], [0], [0], [0, 1]], 5)) == 3
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[3, 1], [0, 4], [3], [0, 2], [1]], 5)) == 4
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[4], [2, 3], [4, 1], [1], [0, 2]], 5)) == 4
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[2, 1, 4], [0], [0, 3], [2], [0]], 5)) == 3
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[3], [4], [3], [0, 2, 4], [3, 1]], 5)) == 3
    assert diameter(Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[1, 3], [0, 2], [1], [0, 4], [3]], 5)) == 4

#testes automatizados para a função bfsMax
def assert_bfsMax():
    G = Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[3], [2], [3, 1], [0, 4, 2], [3]], 5)

    bfsMax(G, G.V[0])

    assert G.V[0].d == 0
    assert G.V[1].d == 3
    assert G.V[2].d == 2
    assert G.V[3].d == 1
    assert G.V[4].d == 2

    g = Grafo([Vertice(i, None, None, 'branco') for i in range(5)], [[4, 2], [4], [0, 3], [2], [0, 1]], 5)

    bfsMax(g, g.V[0])

    assert g.V[0].d == 0
    assert g.V[1].d == 2
    assert g.V[2].d == 1
    assert g.V[3].d == 2
    assert g.V[4].d == 1

#testes automatizados para a função make_set
def assert_make_set():
    G = GrafoKruskal([Vertice(i, None, None, 'branco') for i in range(5)], [[] for i in range(5)] , 5, []) 

    make_set(G.V[0])
    make_set(G.V[1])
    make_set(G.V[2])
    make_set(G.V[3])
    make_set(G.V[4])

    assert G.V[0].pai == G.V[0]
    assert G.V[0].rank == 0
    assert G.V[1].pai == G.V[1]
    assert G.V[1].rank == 0
    assert G.V[2].pai == G.V[2]
    assert G.V[2].rank == 0
    assert G.V[3].pai == G.V[3]
    assert G.V[3].rank == 0
    assert G.V[4].pai == G.V[4]
    assert G.V[4].rank == 0


    assert find_set(G.V[0]) == G.V[0]
    assert find_set(G.V[1]) == G.V[1]
    assert find_set(G.V[2]) == G.V[2]
    assert find_set(G.V[3]) == G.V[3]
    assert find_set(G.V[4]) == G.V[4]

#testes automatizados para a função find_set
def assert_find_set():
    G = GrafoKruskal([Vertice(i, None, None, 'branco') for i in range(5)], [[] for i in range(5)] , 5, []) 

    make_set(G.V[0])
    make_set(G.V[1])
    make_set(G.V[2])
    make_set(G.V[3])
    make_set(G.V[4])

    assert find_set(G.V[0]) != G.V[1]
    assert find_set(G.V[1]) == G.V[1]
    assert find_set(G.V[2]) != G.V[4]
    assert find_set(G.V[3]) == G.V[3]
    assert find_set(G.V[1]) != G.V[4]

#testes automatizados para a função link
def assert_link():
    G = GrafoKruskal([Vertice(i, None, None, 'branco') for i in range(5)], [[] for i in range(5)] , 5, []) 

    make_set(G.V[0])
    make_set(G.V[1])
    make_set(G.V[2])
    make_set(G.V[3])
    make_set(G.V[4])

    link(G.V[0], G.V[3])
    assert G.V[0].pai == G.V[3]
    assert G.V[3].pai == G.V[3]
    assert G.V[1].pai == G.V[1]
    assert G.V[2].pai == G.V[2]


    link(G.V[1], G.V[0])
    assert G.V[1].pai == G.V[0]
    assert G.V[2].pai == G.V[2]
    assert G.V[4].pai == G.V[4]

    G.V[4].rank = 1

    link(G.V[3], G.V[4])
    assert G.V[3].pai == G.V[4]

#testes automatizados para a função uniao
def assert_uniao():
    G = GrafoKruskal([Vertice(i, None, None, 'branco') for i in range(5)], [[] for i in range(5)] , 5, []) 

    make_set(G.V[0])
    make_set(G.V[1])
    make_set(G.V[2])
    make_set(G.V[3])
    make_set(G.V[4])

    uniao(G.V[0], G.V[1])
    assert find_set(G.V[0]) == find_set(G.V[1])
    assert find_set(G.V[0]) != find_set(G.V[2])
    assert find_set(G.V[0]) != find_set(G.V[3])
    assert find_set(G.V[0]) != find_set(G.V[4])

    uniao(G.V[3], G.V[4])
    assert find_set(G.V[3]) == find_set(G.V[4])
    assert find_set(G.V[0]) != find_set(G.V[2])
    assert find_set(G.V[0]) != find_set(G.V[3])
    assert find_set(G.V[0]) != find_set(G.V[4])

#testes automatizados para a função mst_kruskal
def assert_mst_kruskal():
    G = GrafoKruskal  ([], [], 5, [])  
    G.V = [Vertice(i, None, None, 'branco') for i in range(G.num_vertex)]
    G.Adj = [
    [G.V[1]],
    [G.V[0], G.V[2]],
    [G.V[1], G.V[3], G.V[4]],
    [G.V[2], G.V[4]],
    [G.V[2], G.V[3]]
    ]

    G.aresta = [
    [0,1,2],
    [1,2,3],
    [2,3,5],
    [2,4,15],
    [3,4,20]
    ]

    assert mst_kruskal(G) == [[0, 1, 2], [1, 2, 3], [2, 3, 5], [2, 4, 15]]

#testes automatizados para a função extract_min()
def assert_extract_min():
    test1 = [0, 1, 2 ,3, 4]
    assert extract_min(test1) == 0

    test2 = [5, 8, 2 ,9 ,15]
    assert extract_min(test2) == 2

    test3 = [23, 353, 58, 89, 24]
    assert extract_min(test3) == 23

    test4 = [32, 12, 43, 387643, 1]
    assert extract_min(test4) == 1
    

#testes automatizados para a função mst_prim
def assert_mst_prim():
    g = Grafo([Vertice(i, None, None, 'branco') for i in range(5)],
    [[float('inf'), 2, float('inf'), float('inf'), float('inf')],
     [2, float('inf'), 3, float('inf'), float('inf')],
     [float('inf'), 3, float('inf'), 5, 15],
     [float('inf'), float('inf'), 5, float('inf'), 20],
     [float('inf'), float('inf'), 15, 20, float('inf')]], 5)
    #matriz simétrica
    # print(mst_prim(g, g.V[0]))


#função principal onde será calculada a média e escrita posteriormente no arquivo txt
def main():

    #chamada para todos os asserts implementados, ondese o algoritmo executa sem erros de compilação, todos os testes foram um sucesso
    assert_diameter()
    assert_bfsMax()
    assert_find_set()
    assert_link()
    assert_make_set()
    assert_mst_kruskal()
    assert_uniao()
    assert_mst_prim()

    testes = [250, 500, 750, 1000, 1250, 1500, 1750, 2000]

    #### chamada para o random_tree_random_walk ####
    
    #abertura de arquivo para escrita
    file = open("random_tree_random_walk.txt", "w")

    tempo_inicial = time()

    for n in testes:
        tempo_inicial_teste = time()

        print("Teste {}".format(n))
        soma = 0

        for x in range(500):
            soma = soma + diameter(random_tree_random_walk(n))                 
        media = soma/500
        
        tempo_teste = time() - tempo_inicial_teste

        print("N = {}".format(n),"terminou em {}".format(tempo_teste),"segundos")
        #escreve no arquivo
        file.write('{} {}\n'.format(n, media))

    tempo_total = time() - tempo_inicial
    print("Tempo de execução Random Walk: {:.2f}".format(tempo_total), "segundos")

    #### chamada para o random_tree_kruskal ####

    file = open("random_tree_kruskal.txt", "w")

    tempo_inicial = time()

    for n in testes:
        tempo_inicial_teste = time()

        print("Teste {}".format(n))
        soma = 0

        for x in range(500):
            soma = soma + diameter(random_tree_kruskal(n))                 
        media = soma/500
        
        tempo_teste = time() - tempo_inicial_teste

        print("N = {}".format(n),"terminou em {}".format(tempo_teste),"segundos")
        #escreve no arquivo
        file.write('{} {}\n'.format(n, media))

    tempo_total = time() - tempo_inicial
    print("Tempo de execução Kruskal: {:.2f}".format(tempo_total), "segundos")

    file = open("random_tree_prim.txt", "w")

    tempo_inicial = time()

    for n in testes:
        tempo_inicial_teste = time()

        print("Teste {}".format(n))
        soma = 0

        for x in range(500):
            soma = soma + diameter(random_tree_prim(n))                 
        media = soma/500
        
        tempo_teste = time() - tempo_inicial_teste

        print("N = {}".format(n),"terminou em {}".format(tempo_teste),"segundos")
        #escreve no arquivo
        file.write('{} {}\n'.format(n, media))

    tempo_total = time() - tempo_inicial
    print("Tempo de execução Prim: {:.2f}".format(tempo_total), "segundos")

if __name__ == "__main__":
    main()