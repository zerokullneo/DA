// ###### Config options ################

#define PRINT_DEFENSE_STRATEGY 1    // generate map images

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif

#ifdef CUSTOM_RAND_GENERATOR
RAND_TYPE SimpleRandomGenerator::a;
#endif

#ifdef DEFAULT_DEFENSE_HEALTH
#undef DEFAULT_DEFENSE_HEALTH
#endif

#define DEFAULT_DEFENSE_HEALTH std::numeric_limits<float>::max() //(float)UINT16_MAX * 1.0

using namespace Asedio;
enum placePosition{center, border, bordered, other};

class ValueList
{
	public:
		ValueList(float vl, Vector3 ps): value(vl), position(ps){};
		float value;
		Vector3 position;
};
bool operator <(const ValueList &A, const ValueList &B){ return A.value < B.value;}
bool operator >(const ValueList &A, const ValueList &B){ return A.value > B.value;}

/**
 * Dada una celda o posicion devuelve un valor para crear en otra funcion la matriz de valores de las celdas.
 * @param row Fila a evaluar.
 * @param col Columna a evaluar.
 * @param freeCells matriz de numero de celdas-ancho por numero de celdas-alto, contiene true si el centro de la celda
 *                  esta libre y false si el centro de la celda esta ocupado por un obstaculo.
 * @param nCellsWidth numero de celdas en anchura.
 * @param nCellsHeight numero de celdas en altura.
 * @param mapWidth ancho total del mapa preestablecido.
 * @param mapHeight alto total del mapa preestablecido.
 * @param obstacles Lista actual de Obstaculos creados y situados.
 * @param defenses Lista actual de Defensas creadas, no situadas.
 * @return Devuelve un valor determinado para una posicion determinada del tablero.
 */
float cellValue(int row, int col, placePosition place, bool** freeCells, int nCellsWidth, int nCellsHeight
	, float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses)
{
	// implemente aqui la función que asigna valores a las celdas (float)UINT16_MAX

	float valueOfCell = 100.0f;

	switch (place)
	{
		case center:
			valueOfCell = valueOfCell * 1.0f;
			break;
		case border:
			valueOfCell = valueOfCell * 0.9f;
			break;
		case bordered:
			valueOfCell = valueOfCell * 0.5f;
			break;
		case other:
			valueOfCell = valueOfCell * 0.2f;
		default:
			valueOfCell = valueOfCell * 0.1f;
			break;
	}


	return valueOfCell;
}

/**
 * Funcion que crea una matriz de valores correspondientes a cada celda del mapa dados por la funcion cellValue, siendo
 * la funcion Candidatos del algoritmo Voraz
 * @param freeCells matriz de numero de celdas-ancho por numero de celdas-alto, contiene true si el centro de la celda
 *                  esta libre y false si el centro de la celda esta ocupado por un obstaculo.
 * @param nCellsWidth numero de celdas en anchura.
 * @param nCellsHeight numero de celdas en altura.
 * @param mapWidth ancho total del mapa preestablecido.
 * @param mapHeight alto total del mapa preestablecido.
 * @param obstacles Lista actual de Obstaculos creados y situados.
 * @param defenses Lista actual de Defensas creadas, no situadas.
 * @return Devuelve una matriz rellena con los valores de las posiciones candidatas prometedoras.
 */
float** matrixValues(bool** freeCells, int nCellsWidth, int nCellsHeight
, float mapWidth, float mapHeight, const List<Object*> &obstacles, const List<Defense*> &defenses)
{
	float **matrixOfValues; matrixOfValues = new float* [nCellsWidth];
	float MAXV = 100.0f;
	float MEDV = MAXV * 0.9f;

	for (int i = 0; i < nCellsWidth; ++i)
		matrixOfValues[i] = new float [nCellsHeight];

	for(int iW = (nCellsWidth/2)-1; iW < (nCellsWidth/2)+3; iW++)
		for(int iH = (nCellsHeight/2)-1; iH < (nCellsHeight/2)+3; iH++)
			matrixOfValues[iW][iH] = cellValue(iW, iH, placePosition::center, freeCells,nCellsWidth, nCellsHeight, mapWidth,mapHeight, obstacles, defenses);

	/**
	 * for para fila 0 y n
	 */
	for(int iW = 0; iW < nCellsWidth; iW += nCellsWidth-1)
		for(int iH = 0; iH < nCellsHeight; iH++)
			matrixOfValues[iW][iH] = cellValue(iW, iH, placePosition::border, freeCells,nCellsWidth, nCellsHeight, mapWidth,mapHeight, obstacles, defenses);

	/**
	 * for para columna 0 y n
	 */
	for(int iW = 0; iW < nCellsWidth; iW++)
		for(int iH = 0; iH < nCellsWidth; iH += nCellsHeight-1)
			matrixOfValues[iW][iH] = cellValue(iW, iH, placePosition::border, freeCells,nCellsWidth, nCellsHeight, mapWidth,mapHeight, obstacles, defenses);

	for (int iW = 0; iW < nCellsWidth; ++iW)
		for (int iH = 0; iH < nCellsHeight; ++iH)
		{
			if (matrixOfValues[iW][iH] != MAXV and  matrixOfValues[iW][iH] != MEDV)
				matrixOfValues[iW][iH] = cellValue(iW, iH, placePosition::other, freeCells, nCellsWidth, nCellsHeight,
										mapWidth, mapHeight, obstacles, defenses);
		}

	return matrixOfValues;
}

/**
 * Funcion que comprueba la factibilidad de posicionar una defensa en una posicion dada del mapa.
 * @param currentDefense Defensa actual a evaluar su posicion.
 * @param defenses Lista actual de Defensas creadas y situadas.
 * @param obstacles Lista actual de Obstaculos creados y situados.
 * @param mapWidth ancho total del mapa preestablecido.
 * @param mapHeight alto total del mapa preestablecido.
 * @return Devuelve true si se puede colocar y false si no se puede colocar.
 */
bool factibility(Defense* currentDefense, std::list<Defense*> defenses, std::list<Object*> obstacles, float mapWidth, float mapHeight)
{
	/**
	* No esta fuera del ancho del mapa
	*/
	if((currentDefense->position.x - currentDefense->radio) < 0.0 or (currentDefense->position.x + currentDefense->radio) > mapWidth)
		return false;
	/**
	* No esta fuera de la altura del mapa
	*/
	if((currentDefense->position.y - currentDefense->radio) < 0.0 or (currentDefense->position.y + currentDefense->radio) > mapHeight)
		return false;

	List<Object*>::iterator positionObstacle = obstacles.begin();
	List<Defense*>::iterator positionDefense = defenses.begin();
	/**
	* Recorremos la lista de Obstaculos para que no se sobrepongan la defensa actual.
	*/
	for(auto itSearchOb = positionObstacle; itSearchOb != obstacles.end(); ++itSearchOb)
		if( ((*itSearchOb)->radio + currentDefense->radio) > _distance(currentDefense->position, (*itSearchOb)->position) )
			return false;
	/**
	* Recorremos la lista de defensas creadas hasta el momento para que no esten unidas.
	*/
	for(auto itSearchDf = positionDefense; (*itSearchDf) != currentDefense; ++itSearchDf)
		if( ((*itSearchDf)->radio + currentDefense->radio) > _distance(currentDefense->position, (*itSearchDf)->position) )
			return false;

	return true;
}

/**
 * Funcion que selecciona la mejor posicion dado los valores de la matriz cellsvalues.
 * @param valuelist iterador que contiene el valor actual de una posicion dada del mapa.
 * @param freeCells matriz de numero de celdas-ancho por numero de celdas-alto, contiene true si el centro de la celda
 *                  esta libre y false si el centro de la celda esta ocupado por un obstaculo.
 * @param nCellsWidth numero de celdas en anchura.
 * @param nCellsHeight numero de celdas en altura.
 * @param extraction Bit que indica si es o no el centro de extraccion.
 * @return Devuelve un tipo Vector3 con la posicion mas prometedora para la defensa.
 */
Vector3 cellSelect(std::list<ValueList>::iterator valuelist, bool** freeCells, int nCellsWidth, int nCellsHeight, int extraction = 0)
{
	Vector3 valuePosition{};
	float x = valuelist->position.x, y = valuelist->position.y;
	int positionX = (int)valuelist->position.x, positionY = (int)valuelist->position.y;

	if (extraction == 1)
	{
		//::cout << "centro extraccion, positionX: "<< positionX << ", positionY: " << positionY << ". Celda Libre: " << freeCells[positionX][positionY] << std::endl;
		if (freeCells[positionX][positionY] == 1 and (positionX < nCellsWidth and positionY < nCellsHeight))
		{
			valuePosition.x = x;
			valuePosition.y = y;
		}
	}
	else
	{
		//std::cout << "nueva Defensa, positionX: "<< positionX << ", positionY: " << positionY << ". Celda Libre: " << freeCells[positionX][positionY] << std::endl;
		if (freeCells[positionX][positionY] == 1 and (positionX < nCellsWidth and positionY < nCellsHeight))
		{
			valuePosition.x = x;
			valuePosition.y = y;
		}
	}

	return valuePosition;
}


/**
 * Funcion que posiciona las defensas y que tiene implicita la funcion objetivo del algoritmo Voraz
 * @param freeCells matriz de numero de celdas-ancho por numero de celdas-alto, contiene true si el centro de la celda
 *                  esta libre y false si el centro de la celda esta ocupado por un obstaculo.
 * @param nCellsWidth numero de celdas en anchura.
 * @param nCellsHeight numero de celdas en altura.
 * @param mapWidth ancho total del mapa preestablecido.
 * @param mapHeight alto total del mapa preestablecido.
 * @param obstacles Lista actual de Obstaculos creados y situados.
 * @param defenses Lista actual de Defensas creadas, no situadas.
 */
void DEF_LIB_EXPORTED placeDefenses(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses)
{
	float cellWidth = mapWidth / nCellsWidth;
	float cellHeight = mapHeight / nCellsHeight;
	float **cellsValues = new float*[nCellsWidth];
	/**
	 * Matriz de valores del algoritmo voraz.
	 */
	cellsValues = matrixValues(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
	/**
	 * Lista para almacenar la matriz de valores y ordenarlos
	 */
	std::list<ValueList> VectorValues;
	Vector3 v3tmp{}, cellSelection{};
	int maxAttemps = 1000, n = 0;

	for (int i = 0; i < nCellsWidth; ++i)
	{
		for (int j = 0; j < nCellsHeight; ++j)
		{
			v3tmp.x = (float)i;
			v3tmp.y = (float)j;
			VectorValues.emplace_back(cellsValues[i][j], v3tmp);
			//std::cout << cellsValues[i][j] << "\t";
		}
		//std::cout << std::endl;
	}

	/**
	 * Ordenacion de los valores de la lista.
	 */
	VectorValues.sort(std::greater<ValueList>());

	std::list<ValueList>::iterator currentValue = VectorValues.begin();
	/*while (currentValue != VectorValues.end())
	{
		n++;
		std::cout << n << " - Place -- Valor celda: " << (*currentValue).value << ", Posicion celda: ["
				<< (*currentValue).position.x << ", " << (*currentValue).position.y << "]" << std::endl;
		++currentValue;
	}
	 currentValue = VectorValues.begin();*/

	List<Defense*>::iterator currentDefense = defenses.begin();

	while(currentDefense != (defenses.end()) and maxAttemps > 0)
	{
		/**
		* funcion select, primera defensa en el centro
		*/
		if(currentDefense == defenses.begin())
			cellSelection = cellSelect(currentValue, freeCells, nCellsWidth, nCellsHeight, 1);
		else
		{
			cellSelection = cellSelect(currentValue, freeCells, nCellsWidth, nCellsHeight);
		}

		(*currentDefense)->position.x = (cellSelection.x * cellWidth) + (cellWidth * 0.5f); //(int)(_RAND2(nCellsWidth))* cellWidth) + cellWidth * 0.5f
		(*currentDefense)->position.y = (cellSelection.y * cellHeight) + (cellHeight * 0.5f); //(int)(_RAND2(nCellsHeight)) *cellHeight) + cellHeight *0.5f
		(*currentDefense)->position.z = 0;// cellSelection.z;
		if(factibility(*currentDefense, defenses, obstacles, mapWidth, mapHeight))
		{
			//std::cout << "vida defensa " << (*currentDefense)->id << ": " << (*currentDefense)->health << std::endl;
			(*currentDefense)->health = DEFAULT_DEFENSE_HEALTH;
			//std::cout << "vida defensa " << (*currentDefense)->id << ": " << (*currentDefense)->health << std::endl;
			++currentDefense;
			++currentValue;
			//quitar de la lista
			VectorValues.pop_front();
		}
		else
		{
			++currentValue;
			//quitar de la lista
			VectorValues.pop_front();
		}
		//std::cout << VectorValues.size() << " - currentDefense -- Valor celda: " << (*currentValue).value << ", Posicion celda: ["
		//<< (*currentValue).position.x << ", " << (*currentValue).position.y << "]" << std::endl;
	}

#ifdef PRINT_DEFENSE_STRATEGY

	float** cellValues = new float* [nCellsHeight];
	for(int i = 0; i < nCellsHeight; ++i)
	{
		cellValues[i] = new float[nCellsWidth];
		for(int j = 0; j < nCellsWidth; ++j)
		{
			cellValues[i][j] = (int)(cellsValues[i][j]) % 256;
		}
	}
	dPrintMap("defenseValueCellsHead.ppm", nCellsHeight, nCellsWidth, cellHeight, cellWidth, freeCells
			, cellValues, std::list<Defense*>(), true);

	for(int i = 0; i < nCellsHeight ; ++i)
		delete [] cellValues[i];
	delete [] cellValues;
	cellValues = nullptr;

#endif
	for(int i = 0; i < nCellsHeight ; ++i)
		delete [] cellsValues[i];
	delete [] cellsValues;
	cellsValues = nullptr;
}
