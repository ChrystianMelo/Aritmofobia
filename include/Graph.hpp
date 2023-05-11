/**
 * @file Graph.hpp
 * @author Chrystian Melo(meloo.chrys@gmail.com)
 * @brief Classe Graph.
 *
 * Este arquivo cont�m a implementa��o da classe Graph, que representa um grafo.
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

template <typename T>
class Graph
{
private:
	/**
	 * @brief Vetores do grafo.
	 */
	std::vector<T> m_vertices;

	/**
	 * @brief Arestas do grafo.
	 *
	 * Cada aresta � representada por uma tupla (from, to, weight), onde:
	 *		from: v�rtice de origem da aresta
	 *		to: v�rtice de destino da aresta
	 *		weight: peso da aresta
	 */
	std::vector<std::tuple<T, T, int>> m_edges;

	/**
	 * @brief Lista de adjac�ncia representando os v�rtices e as arestas.
	 *
	 * Cada elemento da lista de adjac�ncia � um par (vertex, weight), onde:
	 *		vertex: v�rtice adjacente
	 *		weight: peso da aresta que conecta os v�rtices
	 */
	std::vector<std::vector<std::pair<T, int>>> m_adjacency;

	/**
	 * @brief Remove duplicatas de um vetor.
	 *
	 * @param v Vetor a ser processado.
	 * @return Um novo vetor sem duplicatas.
	 */
	static std::vector<T> equalize(std::vector<T> v)
	{
		for (std::size_t i = 0; i < v.size(); i++)
			for (std::size_t j = 0; j < v.size(); j++)
			{
				if (i != j && v[i] == v[j])
					return {v.begin() + j, v.end()};
			}
		return v;
	}

public:
	/**
	 * @brief Construtor da classe Graph.
	 *
	 * @param verticesNumber N�mero de v�rtices do grafo.
	 */
	Graph(int verticesNumber)
	{
		// Implementa��o do construtor
	}

	/**
	 * @brief Adiciona um v�rtice ao grafo.
	 *
	 * @param vertex V�rtice a ser adicionado.
	 */
	void addVertex(T vertex)
	{
		if (std::find(m_vertices.begin(), m_vertices.end(), vertex) == m_vertices.end())
		{
			m_adjacency.push_back(std::vector<std::pair<T, int>>());
			m_vertices.push_back(vertex);
		}
	}

	/**
	 * @brief Adiciona uma aresta ao grafo.
	 *
	 * @param v1 O primeiro v�rtice da aresta.
	 * @param v2 O segundo v�rtice da aresta.
	 * @param weight O peso da aresta.
	 **/
	void addEdge(T v1, T v2, int weight)
	{
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
	 * @param v1 O primeiro v�rtice da aresta.
	 * @param v2 O segundo v�rtice da aresta.
	 **/
	void removeEdge(T v1, T v2)
	{
		int indexU = getIndex(v1);
		int indexV = getIndex(v2);

		if (indexU == -1 || indexV == -1)
		{
			// Um ou ambos os v�rtices n�o est�o presentes no grafo
			return;
		}

		// Faz a remo��o na lista de adjacencia.
		{
			// Procura a aresta (v1, v2) e a remove
			for (auto it = m_adjacency[indexU].begin(); it != m_adjacency[indexU].end(); ++it)
			{
				if (it->first == v2)
				{
					m_adjacency[indexU].erase(it);
					break;
				}
			}

			// Procura a aresta (v2, v1) e a remove
			for (auto it = m_adjacency[indexV].begin(); it != m_adjacency[indexV].end(); ++it)
			{
				if (it->first == v1)
				{
					m_adjacency[indexV].erase(it);
					break;
				}
			}
		}

		// Faz a remo��o no vetor de arestas.
		{
		}
	}
	/**
	 * @brief Retorna o �ndice do v�rtice no grafo.
	 *
	 * @param vertex O v�rtice a ser adicionado.
	 * @return O �ndice do v�rtice no grafo. Se n�o for encontrado, retorna -1.
	 **/
	int getIndex(T vertex)
	{
		for (std::size_t i = 0; i < m_vertices.size(); i++)
		{
			if (m_vertices[i] == vertex)
			{
				return i;
			}
		}
		return -1;
	}

	/**
	 * @brief Retorna o v�rtice no grafo com base no �ndice fornecido.
	 *
	 * @param index O �ndice do v�rtice.
	 * @return O v�rtice correspondente ao �ndice.
	 **/
	T &getVertex(int index)
	{
		return m_vertices[index];
	}

	/**
	 * @brief Retorna o peso da aresta entre dois v�rtices.
	 *
	 * @param from O primeiro v�rtice da aresta.
	 * @param to O segundo v�rtice da aresta.
	 * @return O peso da aresta entre os v�rtices. Se a aresta n�o existir, retorna 0.
	 **/
	int getWeight(T from, T to)
	{
		int weight = 0;
		for (auto edge : m_edges)
		{
			if (std::get<0>(edge) == from && std::get<1>(edge) == to)
			{
				weight = std::get<2>(edge);
				break;
			}
		}
		return weight;
	}

	/**
	 * @brief Calcula a soma dos pesos ao longo de um caminho.
	 *
	 * @param path Um vetor contendo os v�rtices do caminho.
	 * @return A soma dos pesos dos v�rtices no caminho. Se o caminho for inv�lido, retorna -1.
	 **/
	int calculateWeight(std::vector<T> path)
	{
		if (path.size() < 2)
			return -1;

		int weight = 0;
		if (path.size() == 2)
			weight = getWeight(path.at(0), path.at(1));
		else
		{
			for (std::size_t i = 0; i < path.size() - 1; i++)
			{
				weight += getWeight(path.at(i), path.at(i + 1));
			}
		}

		return weight;
	}

	/**
	 * @brief Implementa o algoritmo de Dijkstra para encontrar o menor caminho entre dois v�rtices.
	 *
	 * @param source O v�rtice de origem.
	 * @param destination O v�rtice de destino.
	 * @param onlyEvenPaths Um flag indicando se apenas caminhos com n�mero par de v�rtices devem ser considerados.
	 * @return Um vetor contendo os v�rtices do caminho m�nimo encontrado.
	 **/
	std::vector<T> dijkstra(T source, T destination, bool onlyEvenPathes = true)
	{
		T originalDestination = destination;
		std::vector<T> path;
		std::vector<int> distances(m_vertices.size(), std::numeric_limits<int>::max());
		std::vector<T> previous(m_vertices.size(), T());

		std::priority_queue<std::pair<int, T>, std::vector<std::pair<int, T>>, std::greater<std::pair<int, T>>> pq;

		int sourceIndex = getIndex(source);
		int destinationIndex = getIndex(destination);

		distances[sourceIndex] = 0;
		pq.push(std::make_pair(0, source));

		while (!pq.empty())
		{
			T currentVertex = pq.top().second;
			int currentDistance = pq.top().first;
			pq.pop();

			if (currentVertex == destination)
			{
				while (previous[destinationIndex] != T())
				{
					path.push_back(destination);
					destination = previous[destinationIndex];
					destinationIndex = getIndex(destination);
				}
				path.push_back(source);
				break;
			}

			int currentVertexIndex = getIndex(currentVertex);

			if (currentDistance > distances[currentVertexIndex])
			{
				continue;
			}

			for (std::pair<T, int> neighbor : m_adjacency[currentVertexIndex])
			{
				int neighborIndex = getIndex(neighbor.first);
				int tentativeDistance = currentDistance + neighbor.second;

				if (tentativeDistance < distances[neighborIndex])
				{
					distances[neighborIndex] = tentativeDistance;
					previous[neighborIndex] = currentVertex;
					pq.push(std::make_pair(tentativeDistance, neighbor.first));
				}
			}
		}

		std::reverse(path.begin(), path.end());

		if (onlyEvenPathes)
		{
			// O caminho tra�cado pelo algoritmo deve passar por um n�umero par de estradas.
			bool research = false;
			while (!path.empty() && (path.size() % 2 == 0 || research))
			{
				research = false;
				// Encontra o �ltimo v�rtice com outro caminho dispon�vel.
				int lastValidIndex = -1;
				for (std::size_t i = path.size() - 2; i >= 0; --i)
				{
					int vertexIndex = getIndex(path[i]);
					for (std::pair<T, int> neighbor : m_adjacency[vertexIndex])
					{
						int neighborIndex = getIndex(neighbor.first);
						if (distances[neighborIndex] + neighbor.second == distances[vertexIndex])
						{
							lastValidIndex = i;
							break;
						}
					}
					if (lastValidIndex != -1)
						break;
				}

				if (lastValidIndex != -1)
				{
					auto a = path.at(lastValidIndex);
					auto b = path.at(lastValidIndex + 1);
					int weight = getWeight(a, b);

					removeEdge(a, b);
					path.erase(path.begin() + lastValidIndex + 1, path.end());

					std::vector<T> path2 = dijkstra(path.at(path.size() - 1), originalDestination, false);
					if (path2.empty())
					{
						addEdge(a, b, weight);
						research = true;
					}
					else
					{
						path.insert(path.end(), path2.begin() + 1, path2.end());
						path = equalize(path);
					}
				}
				else
					break;
			}
		}
		return path;
	}
};
#endif