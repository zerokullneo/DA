// ###### Config options ################


// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

using namespace Asedio;

/**
 * Función que dada una defensa devuelva su capacidad o valor defensivo. La capacidad o valor defensiva
 * es una valoracion de los atributos health y cost mas attacksPerSecond y damage mas range.
 * @param currentDefense Iterador a la defensa actual para calcular su valor.
 * @return Devuelve un valor tipo float simulando la capacidad defensiva de la defensa.
 */
float defensiveCapacity(std::list<Defense*>::iterator currentDefense)
{
	/*std::cout << "idDefensa: " << (*currentDefense)->id << " - Vida/Capacidad: " << (*currentDefense)->health
			<< ", coste defensa: " << (*currentDefense)->cost << ", AtaquesxSecond: " << (*currentDefense)->attacksPerSecond
			<< ", Daño producido: " << (*currentDefense)->damage << ", Rango: " << (*currentDefense)->range << std::endl;*/
	return ((*currentDefense)->health / (float)(*currentDefense)->cost)
			+ ((*currentDefense)->attacksPerSecond + (*currentDefense)->damage)
			+ ((*currentDefense)->range / 10);
}

/**
 * Funcion  que rellena la tabla de subproblemas resueltos con el algoritmo de la mochila
 * @param defenses Lista de defensas disponibles para adquirir.
 * @param ases Cantidad de ases/monedas para adquirir las defensas.
 * @return Devuelve la tabla de subproblemas resueltos en forma de matriz, con forma de vector de vectores.
 */
std::vector< std::vector<float> > tableResults(std::list<Defense*> defenses, unsigned int ases)
{
	size_t tlpWidth = defenses.size();//objetos
	size_t tlpHeight = ases;//capacidad
	std::vector< std::vector<float> > defensesAses(tlpWidth, std::vector<float>(tlpHeight));
	float value;

	std::list<Defense*>::iterator itDefenses = defenses.begin();

	/**
	 * valor = ((*currentDefense)->health / (float)(*currentDefense)->cost) + ((*currentDefense)->attacksPerSecond + (*currentDefense)->damage)
	 * peso = (*itDefenses)->cost
	 * salttamos de la lista de defensas la primera que ya ha sido comprada.
	 */
	++itDefenses;
	for (int j = 0; j < tlpHeight; ++j)
	{
		if ( j < (*itDefenses)->cost )
			defensesAses[0][j] = 0;
		else
			defensesAses[0][j] = defensiveCapacity(itDefenses);
	}

	for (int i = 1; i < tlpWidth; ++i, ++itDefenses)
	{
		value = defensiveCapacity(itDefenses);
		for (int j = 0; j < tlpHeight; ++j)
		{
			if (j < (*itDefenses)->cost)
				defensesAses[i][j] = defensesAses[i-1][j];
			else
				defensesAses[i][j] = std::max(defensesAses[i-1][j], defensesAses[i-1][j - (*itDefenses)->cost] + value);
		}
	}
/*
	for (int i = 0; i < tlpWidth; ++i)
	{
		for (int j = 0; j < tlpHeight; ++j)
		{
			std::cout << defensesAses[i][j] << " ";
		}
		std::cout << std::endl;
	}
*/
	return defensesAses;
}

/**
 * Función que recupera la solucion de la tabla rellena, almacenando los "id" "selectedIDs"
 * @param tlpResults Matriz o tabla de subproblemas resueltos previamente calculada.
 * @param defenses Lista de defensas disponibles para adquirir.
 * @param selectedIDs Paso por referencia de la lista de IDs seleccionado
 * @return Devuelve la lista de IDs seleccionados en selectedIDs
 */
std::vector<float> pathResult(std::vector< std::vector<float> > tlpResults, std::list<Defense*> defenses, std::list<int> &selectedIDs)
{
	int tlpWidth = (int)tlpResults.size();
	int tlpHeight = (int)tlpResults.data()->size();
	std::vector<float> pathSolution(tlpWidth);
	//std::vector<int> defensesIDs(tlpWidth), defensesCosts(tlpWidth);

	auto itDefenses = --defenses.end();

	for (int i = tlpWidth-1; i > 0; --i, --itDefenses)
	{
		for (int j = tlpHeight-1; j > 0; j = j - (*itDefenses)->cost)
		{
			if (tlpResults[i][j] != tlpResults[i - 1][j])
			{
				pathSolution[i] = tlpResults[i][j];
				selectedIDs.push_back((*itDefenses)->id);
			}
			else
				if (i == 1 and (tlpResults[i][j] == tlpResults[i-1][j]))
				{
					pathSolution[i] = tlpResults[1][j];
					selectedIDs.push_back((*itDefenses)->id);
				}
		}
	}
	/*std::cout << "pathSolution: " << pathSolution.size() << std::endl;
	for (size_t ii = 0; ii < tlpWidth; ++ii)
		std::cout << pathSolution[ii] << " ";
	std::cout << std::endl;*/
	return pathSolution;
}

/**
 * Hace las funciones de algoritmo de la mochila
 * @param defenses Lista de defensas disponibles.
 * @param ases Cantidad monetaria disponible para comprar defennsas.
 * @param selectedIDs Lista de identificadodes de las defensas seleccionadas.
 * @param mapWidth ancho total del mapa preestablecido.
 * @param mapHeight alto total del mapa preestablecido.
 * @param obstacles Lista actual de Obstaculos creados y situados.
 */
void DEF_LIB_EXPORTED selectDefenses(std::list<Defense*> defenses, unsigned int ases, std::list<int> &selectedIDs
				 , float mapWidth, float mapHeight, std::list<Object*> obstacles)
{
	size_t tlpWidth = defenses.size();
	size_t tlpHeight = defenses.size();
	/*
	 * crear tabla y estructura de subproblemas resueltos, con los valores y pesos, aplicando el algoritmo de la mochila
	 * codigo del proceso de recuperacion de una tabla de subproblemas resueltos.
	 */
	std::vector< std::vector<float> > pathSolution(tlpWidth, std::vector<float>(tlpHeight));

	unsigned int cost = 0;
	std::list<Defense*>::iterator it = defenses.begin();

	/**
	 * Insertamos en la lista de seleccionados el centro de extraccion
	 */
	selectedIDs.push_back((*it)->id);

	/**
	 * restamos a "ases" el coste del centro de extraccion
	 */
	ases -= (*it)->cost;
	it++;

	pathSolution = tableResults(defenses, ases);
	pathResult(pathSolution, defenses, selectedIDs);
}