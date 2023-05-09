/**
 * @file Graph.hpp
 * @author Chrystian Melo (meloo.chrys@gmail.com)
 * @brief classe Graph.
 *
 * @copyright Copyright (c) 2023
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
		\brief Vertices do grafo.
	**/
	std::vector<T> m_vertices;

	/**
		\brief Vertices do grafo.
				(T from, T to, int weight)

	**/
	std::vector<std::tuple<T, T, int>> m_edges;

	/**
		\brief Lista de adjacencia representando os vertices e as arestas.
	**/
	std::vector<std::vector<std::pair<T, int>>> m_adjacency;
public:
	/**
		\brief Construtor da classe Graph.

		\param verticesNumber Número de vértices do grafo.
	**/
	Graph(int verticesNumber)
	{
	}

	/**
		\brief Adiciona um vertice ao grafo.

		\param vertexVértice.
	**/
	void addVertex(T vertex) {
		if (std::find(m_vertices.begin(), m_vertices.end(), vertex) == m_vertices.end()) {
			m_adjacency.push_back(std::vector<std::pair<T, int>>());
			m_vertices.push_back(vertex);
		}
	}

	/**
		\brief Adiciona uma aresta ao grafo.

		\param v1 Primeiro vértice da aresta.
		\param v2 Segundo vértice da aresta.
		\param weight Peso da aresta.
	**/
	void addEdge(T v1, T v2, int weight) {
		int indexU = getIndex(v1);
		int indexV = getIndex(v2);
		m_adjacency[indexU].push_back(std::make_pair(v2, weight));
		m_adjacency[indexV].push_back(std::make_pair(v1, weight));

		// Toda aresta é ida e volta.
		m_edges.push_back(std::make_tuple(v1, v2, weight));
		m_edges.push_back(std::make_tuple(v2, v1, weight));
	}

	/**
		\brief Recupera o indice do vertice no grafo.
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
		\brief Recupera o vertice do grafo.
	**/
	T& getVertex(int index) {
		return m_vertices[index];
	}

	/**
		\brief Calcula a soma dos pesos seguindo o caminho.
	**/
	int calculateWeight(std::vector<T> path) {
		if (path.size() < 2) return -1;

		int weight = 0;
		if (path.size() == 2) {
			for (auto edge : m_edges) {
				if (std::get<0>(edge) == path.at(0) && std::get<1>(edge) == path.at(1)) {
					weight = std::get<2>(edge);
					break;
				}
			}
		}
		else {
			for (int i = 0; i < path.size() - 1; i++) {
				for (auto edge : m_edges) {
					if (std::get<0>(edge) == path.at(i) && std::get<1>(edge) == path.at(i + 1)) {
						weight += std::get<2>(edge);
						break;
					}
				}
			}
		}

		return weight;
	}

	/**
		\brief Algoritmo de Dijkstra para encontrar o menor caminho entre dois vértices.

		\param source Vértice de origem.
		\param destination Vértice de destino.

		\return Vetor contendo os vértices do caminho mínimo encontrado.
	**/
	std::vector<T> dijkstra(T source, T destination) {
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
		return path;
	}

};
#endif