/**
 * @file Main.cc
 * @author Chrystian Melo (meloo.chrys@gmail.com)
 * @brief Classe principal do projeto.
 *
 * @copyright Copyright (c) 2023
 */
#include <iostream>
#include <cassert>
#include <fstream>

#include "Graph.hpp"

/**
 * @brief Verifca se o numero � primo.
 **/
bool isOdd(int number)
{
	return number % 2 != 0;
}

/**
 * @brief Classe principal do projeto.
 **/
int main(int argc, char const *argv[])
{
	// assert(argc == 1);
	std::string inputFilename(argv[0]);
	// std::string inputFilename("C:\\Users\\Chrystian Melo\\Documents\\Alg1\\Aritmofobia\\test_cases\\inputs\\test_case9.txt");

	std::ifstream myfileInput(inputFilename);
	assert(myfileInput);

	int verticesNumber, edgesNumber;
	myfileInput >> verticesNumber >> edgesNumber;

	Graph<int> graph(verticesNumber);

	// Adiciona os vertices e arestas.
	for (int i = 0; i < edgesNumber; i++)
	{
		int from, to, weight;
		myfileInput >> from >> to >> weight;

		// Steve s� viaja entre duas cidades adjacentes se a estrada que conecta as duas cidades tiver comprimento par;
		if (!isOdd(weight))
		{
			graph.addVertex(from);
			graph.addVertex(to);

			graph.addEdge(from, to, weight);
		}
	}

	// (2) a cidade de origem sempre recebe o identificador 1
	int searchSource = 1;
	// (3) a cidade de destinosempre recebe o identificador N
	int searchDestination = verticesNumber;

	// Faz a busca do melhor caminho.
	std::vector<int> shortestPath = graph.dijkstra(searchSource, searchDestination);

	// O caminho tra�cado pelo algoritmo deve passar por um n�umero par de estradas.
	int weight = (!shortestPath.empty() && isOdd((int)shortestPath.size())) ? graph.calculateWeight(shortestPath) : -1;

	std::cout << weight << std::endl;

	return 0;
}