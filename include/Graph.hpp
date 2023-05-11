/**
* @file Graph.hpp
* @author Chrystian Melo(meloo.chrys@gmail.com)
* @brief Classe Graph.
*
* Este arquivo contém a implementação da classe Graph, que representa um grafo.
*/

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <utility>
#include <functional>
#include <vector>
#include <limits>
#include <queue>
#include <cassert>

template<typename T>
class Graph {
private:
	/**
	 * @brief Vetores do grafo.
	 */
	std::vector<T> m_vertices;

	/**
	 * @brief Arestas do grafo.
	 *
	 * Cada aresta é representada por uma tupla (from, to, weight), onde:
	 *		from: vértice de origem da aresta
	 *		to: vértice de destino da aresta
	 *		weight: peso da aresta
	 */
	std::vector<std::tuple<T, T, int>> m_edges;

	/**
	 * @brief Lista de adjacência representando os vértices e as arestas.
	 *
	 * Cada elemento da lista de adjacência é um par (vertex, weight), onde:
	 *		vertex: vértice adjacente
	 *		weight: peso da aresta que conecta os vértices
	 */
	std::vector<std::vector<std::pair<T, int>>> m_adjacency;

	/**
	 * @brief Obtém o índice de um elemento em um vetor.
	 *
	 * @param v Vetor a ser pesquisado.
	 * @param value Valor a ser encontrado.
	 * @return O índice do valor no vetor ou -1 se não for encontrado.
	 */
	template<typename T>
	static int getIndexFromVet(std::vector<T> v, T value)
	{
		auto it = std::find(v.begin(), v.end(), value);
		int index = -1;

		if (it != v.end())
			index = it - v.begin();

		return index;
	}

	/**
	 * @brief Remove duplicatas de um vetor.
	 *
	 * @param v Vetor a ser processado.
	 * @return Um novo vetor sem duplicatas.
	 */
	template<typename T>
	static std::vector < T> equalize(std::vector<T> v)
	{
		for (int i = 0; i < v.size(); i++)
			for (int j = 0; j < v.size(); j++) {
				if (i != j && v[i] == v[j])
					return { v.begin() + j, v.end() };
			}
		return v;
	}

public:
	/**
	 * @brief Construtor da classe Graph.
	 *
	 * @param verticesNumber Número de vértices do grafo.
	 */
	Graph(int verticesNumber)
	{
		// Implementação do construtor
	}

	/**
	 * @brief Adiciona um vértice ao grafo.
	 *
	 * @param vertex Vértice a ser adicionado.
	 */
	void addVertex(T vertex) {
		if (std::find(m_vertices.begin(), m_vertices.end(), vertex) == m_vertices.end()) {
			m_adjacency.push_back(std::vector<std::pair<T, int>>());
			m_vertices.push_back(vertex);
		}
	}

	/**
	 * @brief Adiciona uma aresta ao grafo.
	 *
	 * @param v1 O primeiro vértice da aresta.
	 * @param v2 O segundo vértice da aresta.
	 * @param weight O peso da aresta.
	**/
	void addEdge(T v1, T v2, int weight) {
		int indexU = getIndex(v1);
		int indexV = getIndex(v2);
		m_adjacency[indexU].push_back(std::make_pair(v2, weight));
		m_adjacency[indexV].push_back(std::make_pair(v1, weight));

		// Adiciona a aresta de ida e volta.
		m_edges.push_back(std::make_tuple(v1, v2, weight));
		m_edges.push_back(std::make_tuple(v2, v1, weight));
	}

	/**
	 * @brief Remove uma aresta do grafo.
	 *
	 * @param v1 O primeiro vértice da aresta.
	 * @param v2 O segundo vértice da aresta.
	**/
	void removeEdge(T v1, T v2) {
		int indexU = getIndex(v1);
		int indexV = getIndex(v2);

		if (indexU == -1 || indexV == -1) {
			// Um ou ambos os vértices não estão presentes no grafo
			return;
		}

		// Faz a remoção na lista de adjacencia.
		{
			// Procura a aresta (v1, v2) e a remove
			for (auto it = m_adjacency[indexU].begin(); it != m_adjacency[indexU].end(); ++it) {
				if (it->first == v2) {
					m_adjacency[indexU].erase(it);
					break;
				}
			}

			// Procura a aresta (v2, v1) e a remove
			for (auto it = m_adjacency[indexV].begin(); it != m_adjacency[indexV].end(); ++it) {
				if (it->first == v1) {
					m_adjacency[indexV].erase(it);
					break;
				}
			}
		}

		// Faz a remoção no vetor de arestas.
		{
		}
	}
	/**
	 * @brief Retorna o índice do vértice no grafo.
	 *
	 * @param vertex O vértice a ser adicionado.
	 * @return O índice do vértice no grafo. Se não for encontrado, retorna -1.
	**/
	int getIndex(T vertex) {
		for (int i = 0; i < m_vertices.size(); i++) {
			if (m_vertices[i] == vertex) {
				return i;
			}
		}
		return -1;
	}

	/**
	 * @brief Retorna o vértice no grafo com base no índice fornecido.
	 *
	 * @param index O índice do vértice.
	 * @return O vértice correspondente ao índice.
	**/
	T& getVertex(int index) {
		return m_vertices[index];
	}

	/**
	 * @brief Retorna o peso da aresta entre dois vértices.
	 *
	 * @param from O primeiro vértice da aresta.
	 * @param to O segundo vértice da aresta.
	 * @return O peso da aresta entre os vértices. Se a aresta não existir, retorna 0.
	**/
	int getWeight(T from, T to) {
		int weight = 0;
		for (auto edge : m_edges) {
			if (std::get<0>(edge) == from && std::get<1>(edge) == to) {
				weight = std::get<2>(edge);
				break;
			}
		}
		return weight;
	}

	/**
	 * @brief Calcula a soma dos pesos ao longo de um caminho.
	 *
	 * @param path Um vetor contendo os vértices do caminho.
	 * @return A soma dos pesos dos vértices no caminho. Se o caminho for inválido, retorna -1.
	**/
	int calculateWeight(std::vector<T> path) {
		if (path.size() < 2) return -1;

		int weight = 0;
		if (path.size() == 2)
			weight = getWeight(path.at(0), path.at(1));
		else {
			for (int i = 0; i < path.size() - 1; i++) {
				weight += getWeight(path.at(i), path.at(i + 1));
			}
		}

		return weight;
	}

	/**
	 * @brief Implementa o algoritmo de Dijkstra para encontrar o menor caminho entre dois vértices.
	 *
	 * @param source O vértice de origem.
	 * @param destination O vértice de destino.
	 * @param onlyEvenPaths Um flag indicando se apenas caminhos com número par de vértices devem ser considerados.
	 * @return Um vetor contendo os vértices do caminho mínimo encontrado.
	**/
	std::vector<T> dijkstra(T source, T destination, bool onlyEvenPathes = true) {
		T originalDestination = destination;
		std::vector<T> path;
		std::vector<int> distances(m_vertices.size(), std::numeric_limits<int>::max());
		std::vector<T> previous(m_vertices.size(), T());

		std::priority_queue<std::pair<int, T>, std::vector<std::pair<int, T>>, std::greater<std::pair<int, T>>> pq;

		int sourceIndex = getIndex(source);
		int destinationIndex = getIndex(destination);

		distances[sourceIndex] = 0;
		pq.push(std::make_pair(0, source));

		while (!pq.empty()) {
			T currentVertex = pq.top().second;
			int currentDistance = pq.top().first;
			pq.pop();

			if (currentVertex == destination) {
				while (previous[destinationIndex] != T()) {
					path.push_back(destination);
					destination = previous[destinationIndex];
					destinationIndex = getIndex(destination);
				}
				path.push_back(source);
				break;
			}

			int currentVertexIndex = getIndex(currentVertex);

			if (currentDistance > distances[currentVertexIndex]) {
				continue;
			}

			for (std::pair<T, int> neighbor : m_adjacency[currentVertexIndex]) {
				int neighborIndex = getIndex(neighbor.first);
				int tentativeDistance = currentDistance + neighbor.second;

				if (tentativeDistance < distances[neighborIndex]) {
					distances[neighborIndex] = tentativeDistance;
					previous[neighborIndex] = currentVertex;
					pq.push(std::make_pair(tentativeDistance, neighbor.first));
				}
			}
		}

		std::reverse(path.begin(), path.end());

		if (onlyEvenPathes) {
			// O caminho tra¸cado pelo algoritmo deve passar por um n´umero par de estradas.
			bool research = false;
			while (!path.empty() && (path.size() % 2 == 0 || research)) {
				research = false;
				// Encontra o último vértice com outro caminho disponível.
				std::size_t  lastValidIndex = -1;
				for (std::size_t i = path.size() - 2; i >= 0; --i) {
					int vertexIndex = getIndex(path[i]);
					for (std::pair<T, int> neighbor : m_adjacency[vertexIndex]) {
						int neighborIndex = getIndex(neighbor.first);
						if (distances[neighborIndex] + neighbor.second == distances[vertexIndex]) {
							lastValidIndex = i;
							break;
						}
					}
					if (lastValidIndex != -1)
						break;

				}

				if (lastValidIndex != -1) {
					auto a = path.at(lastValidIndex);
					auto b = path.at(lastValidIndex + 1);
					int weight = getWeight(a, b);

					removeEdge(a, b);
					path.erase(path.begin() + lastValidIndex + 1, path.end());

					std::vector<T> path2 = dijkstra(path.at(path.size() - 1), originalDestination, false);
					if (path2.empty()) {
						addEdge(a, b, weight);
						research = true;
					}
					else {
						path.insert(path.end(), path2.begin() + 1, path2.end());
						path = equalize<T>(path);
					}
				}
				else break;
			}
		}
		return path;
	}


};
#endif