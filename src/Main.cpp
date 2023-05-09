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
	\brief 
**/
bool isEven(int number) {
	return number % 2 == 0;
}

/**
	\brief
**/
int main(int argc, char const* argv[]){
	assert(argc == 2);
	std::string inputFilename(argv[0]);
	std::string outputFilename(argv[1]);
	//std::string inputFilename("C:\\Users\\Chrystian Melo\\Documents\\Alg1\\Aritmofobia\\test_cases\\inputs\\test_case4.txt");
	//std::string outputFilename("output.sol");

	std::ifstream myfileInput(inputFilename);
	assert(myfileInput);

	int verticesNumber, edgesNumber;
	myfileInput >> verticesNumber >> edgesNumber;

	Graph<int> graph(verticesNumber);


	// Adiciona os vertices e arestas.
	for (int i = 0; i < edgesNumber; i++) {
		int from, to, weight;
		myfileInput >> from >> to >> weight;

		// Só adiciona vertice no grafo se o peso for ímpar.
		if (isEven(weight)) {
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

	// O caminho tra¸cado pelo algoritmo deve passar por um n´umero par de estradas.
	assert(isEven(shortestPath.size()));

	std::cout << "Shortest path from " << searchSource << " to " << searchDestination << ": ";
	for (const auto& d : shortestPath) {
		std::cout << d << " ";
	}
	std::cout << std::endl;
	std::cout << "Shortest path weight is " << graph.calculateWeight(shortestPath) << std::endl;


	std::ofstream myfileOutput(outputFilename);
	assert(myfileOutput);
	myfileOutput << graph.calculateWeight(shortestPath) << std::endl;

	return 0;
}