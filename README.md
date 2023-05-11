# Aritmofobia

# Introdução
Steven Jodds, um empreendedor de sucesso, enfrenta um desafio incomum em sua vida: uma aversão intensa a números ímpares. Apesar dessa peculiaridade, ele conseguiu construir uma carreira próspera e viajar pelo país para encontrar clientes. No entanto, seu medo de voar o obriga a realizar todas as viagens de negócios de carro. Agora, ele busca criar um algoritmo que o auxilie a planejar suas viagens de forma mais eficiente, levando em consideração suas restrições numéricas.
O objetivo principal é desenvolver um algoritmo capaz de determinar a rota mais curta entre duas cidades, utilizando um grafo que representa a malha de cidades e estradas da região. No entanto, o algoritmo deve considerar as particularidades de Steven:
Restrição de comprimento: Steven só concorda em viajar entre duas cidades vizinhas se o comprimento da estrada que as conecta for um número par. Isso ocorre devido à sua aversão a números ímpares, e garantir que o comprimento seja par é essencial para o seu conforto durante a viagem.
Restrição do número de estradas: O trajeto selecionado pelo algoritmo deve incluir um número par de estradas percorridas. Essa condição é necessária para atender à preferência de Steven por equilíbrio e simetria numérica.
A ideia do projeto é criar um algoritmo personalizado que leve em consideração suas particularidades e o ajude a planejar suas viagens de forma eficiente, garantindo rotas que respeitem suas restrições numéricas e ofereçam uma experiência mais tranquila e satisfatória para ele.

# Modelagem
O modelo de grafos adotado nessa implementação é baseado em uma representação por lista de adjacência. Nesse modelo, cada vértice do grafo é representado por um elemento em um vetor, e cada elemento contém uma lista de pares, onde cada par representa uma aresta do grafo, contendo o vértice adjacente e o peso da aresta.
A escolha desse modelo e das estruturas de dados utilizadas tem suas justificativas:
Lista de Adjacência: A representação por lista de adjacência é uma escolha comum quando se trabalha com grafos esparsos, ou seja, grafos com poucas arestas em relação ao número de vértices. Nesse modelo, é mais eficiente em termos de uso de memória e tempo de execução para acessar e percorrer as arestas de um vértice.
Vetor de Vértices: Utilizar um vetor para armazenar os vértices do grafo permite acesso rápido aos vértices por meio de seus índices, facilitando a implementação de certas operações.
Vetor de Arestas: O vetor de arestas é utilizado para armazenar todas as arestas do grafo. Isso permite uma manipulação mais direta das arestas, como adicionar ou remover uma aresta específica.
Fila de Prioridade (Priority Queue): A fila de prioridade é utilizada no algoritmo de Dijkstra para selecionar o vértice com a menor distância a cada iteração. A escolha de uma fila de prioridade baseada em heap permite que a seleção seja eficiente, garantindo uma complexidade assintótica mais baixa.
Em resumo, a escolha do modelo de grafos por lista de adjacência e das estruturas de dados como vetores e fila de prioridade é motivada pelo desempenho e eficiência para as operações realizadas nos algoritmos implementados, como adicionar vértices, adicionar e remover arestas, encontrar o menor caminho e calcular pesos.

# Implementação
Adicionar vértice (addVertex):
Complexidade: O(1)
A operação de adicionar um vértice envolve apenas a inserção de um elemento no final do vetor de vértices. Essa operação é realizada em tempo constante.
Adicionar aresta (addEdge):
Complexidade: O(1)
A operação de adicionar uma aresta envolve a adição de dois pares (vértice adjacente e peso) nas listas de adjacência dos vértices envolvidos. Essa operação é realizada em tempo constante.
Remover aresta (removeEdge):
Complexidade: O(E)
A operação de remover uma aresta envolve a busca e remoção do par correspondente nas listas de adjacência dos vértices envolvidos. Essa operação tem complexidade linear em relação ao número de arestas do grafo.
Encontrar o índice de um vértice (getIndex):
Complexidade: O(V)
A operação de encontrar o índice de um vértice envolve percorrer o vetor de vértices até encontrar o vértice desejado. Essa operação tem complexidade linear em relação ao número de vértices do grafo.
Calcular o peso de uma aresta (getWeight):
Complexidade: O(E)
A operação de calcular o peso de uma aresta envolve percorrer o vetor de arestas até encontrar a aresta desejada. Essa operação tem complexidade linear em relação ao número de arestas do grafo.
Algoritmo de Dijkstra (dijkstra):
Complexidade: O((V + E) * log(V))
O algoritmo de Dijkstra utiliza uma fila de prioridade (priority queue) baseada em heap para selecionar o vértice com a menor distância a cada iteração. A operação de inserção e remoção na fila de prioridade tem complexidade logarítmica (log(V)). Como o algoritmo executa V iterações, a complexidade final é (V + E) * log(V), onde V é o número de vértices e E é o número de arestas do grafo.

