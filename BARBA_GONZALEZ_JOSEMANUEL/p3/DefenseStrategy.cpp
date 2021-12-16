// ###### Config options ################



// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1
#define CALC_DEFENSE_STRATEGY 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include "cronometro.h"

using namespace Asedio;         

const double e_abs = 0.01,
		e_rel = 0.001;
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

float defaultCellValue(int row, int col, bool** freeCells, int nCellsWidth, int nCellsHeight
    , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses) {
    	
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    Vector3 cellPosition((col * cellWidth) + cellWidth * 0.5f, (row * cellHeight) + cellHeight * 0.5f, 0);
    	
    float val = 0;
    for (List<Object*>::iterator it=obstacles.begin(); it != obstacles.end(); ++it) {
	    val += _distance(cellPosition, (*it)->position);
    }

    return val;
}

#ifdef CALC_DEFENSE_STRATEGY
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
			matrixOfValues[iW][iH] = defaultCellValue(iW, iH, freeCells, nCellsWidth, nCellsHeight, mapWidth,
											  mapHeight, obstacles, defenses);

	/**
	 * for para fila 0 y n
	 */
	for(int iW = 0; iW < nCellsWidth; iW += nCellsWidth-1)
		for(int iH = 0; iH < nCellsHeight; iH++)
			matrixOfValues[iW][iH] = defaultCellValue(iW, iH, freeCells, nCellsWidth, nCellsHeight, mapWidth,
											  mapHeight, obstacles, defenses);

	/**
	 * for para columna 0 y n
	 */
	for(int iW = 0; iW < nCellsWidth; iW++)
		for(int iH = 0; iH < nCellsWidth; iH += nCellsHeight-1)
			matrixOfValues[iW][iH] = defaultCellValue(iW, iH, freeCells,nCellsWidth, nCellsHeight, mapWidth,
											  mapHeight, obstacles, defenses);

	for (int iW = 0; iW < nCellsWidth; ++iW)
		for (int iH = 0; iH < nCellsHeight; ++iH)
		{
			if (matrixOfValues[iW][iH] != MAXV and  matrixOfValues[iW][iH] != MEDV)
				matrixOfValues[iW][iH] = defaultCellValue(iW, iH, freeCells, nCellsWidth, nCellsHeight,
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
#endif/* P1 */

/**
 *
 * @param v
 * @param i
 * @param k
 * @param j
 */
void Fusion(std::vector<ValueList>& v, size_t i, size_t k, size_t j)
{
	size_t n = j - i + 1;
	size_t p = i; size_t q = k + 1;
	std::vector<ValueList> w;

	for(int it = 0; it < n; ++it)
	{
		if (p <= k and (q > j or v[p].value <= v[q].value))
		{
			w.push_back(v[p]);
			p++;
		}
		else
		{
			w.push_back(v[q]);
			q++;
		}
	}
	for (int it = 0; it < n; ++it)
		v[i - 1 + it] = w[it];
}

/**
 *
 * @param orderCells
 * @param pos_i
 * @param pos_j
 */
void orderFusion(std::vector<ValueList>& orderCells, size_t pos_i, size_t pos_j)
{
	size_t n = pos_j - pos_i + 1;
	size_t n0 = 3, pos_k;

	if (n <= n0)
        std::sort(orderCells.begin() + pos_i, orderCells.begin() + pos_j, std::greater<ValueList>());
	else
	{
		pos_k = pos_i - 1 + n / 2;
		orderFusion(orderCells, pos_i, pos_k);
		orderFusion(orderCells, pos_k + 1, pos_j);
		Fusion(orderCells, pos_i, pos_k, pos_j);
	}
}

/* ORDENACION RAPIDA DA
	**
 *
 * @param orderCells
 * @param pos_i
 * @param pos_j
 * @return
 *
size_t pivote(std::vector<ValueList>& orderCells, size_t pos_i, size_t pos_j)
{
	size_t p = pos_i;
	float x = orderCells[pos_i];

	for (int k = pos_i + 1; k < pos_j; ++k)
	{
		if (orderCells[k] <= x)
		{
			p = p + 1;
			float aux = orderCells[p];
			orderCells[p] = orderCells[k];
			orderCells[k] = aux;
		}
		orderCells[pos_i] = orderCells[p];
		orderCells[p] = x;
	}
}

**
 *
 * @param orderCells
 * @param pos_i
 * @param pos_j
 *
void orderRapida(std::vector<ValueList>& orderCells, size_t pos_i, size_t pos_j)
{
	size_t n = pos_j - pos_i + 1;
	size_t n0 = 3;

	if(n <= n0)
        std::sort(orderCells.begin() + pos_i, orderCells.begin() + pos_j + 1, std::greater<ValueList>());
	else
	{
		size_t p = pivote(orderCells, pos_i, pos_j);
		orderRapida(orderCells, pos_i, p - 1);
		orderRapida(orderCells, p + 1, pos_j);
	}
}*/

void quickSort(std::vector<ValueList>& orderCells, size_t izq, size_t dch)
{
	std::vector<ValueList> aux;
    float piv;
	size_t i=izq, d=dch;

	if (izq >= dch) return;
	piv = orderCells[(izq + dch) / 2].value;

	while (i < d)
	{
		for (; orderCells[i].value < piv; ++i);
		for (; orderCells[d].value > piv; --d);
		if (i <= d)
		{
            std::swap(orderCells[i], orderCells[d]);
			/*aux = orderCells[i];
			orderCells[i] = orderCells[d];
			orderCells[d] = aux;*/
			++i;
			--d;
		}
	}

	quickSort(orderCells, izq, d);
	quickSort(orderCells, i, dch);
}

void DEF_LIB_EXPORTED placeDefensesNoOrdenacion(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
		, List<Object*> obstacles, List<Defense*> defenses)
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
	//VectorValues.sort(std::greater<ValueList>());

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
	//delete [] cellsValues;
	cellsValues = nullptr;
}

void DEF_LIB_EXPORTED placeDefensesFusion(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
		, List<Object*> obstacles, List<Defense*> defenses)
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
	std::vector<ValueList> orderCells(VectorValues.begin(), VectorValues.end());
    orderFusion(orderCells, orderCells.size()-orderCells.size(), orderCells.size());
	//VectorValues.sort(std::greater<ValueList>());

	std::list<ValueList>::iterator currentValue = VectorValues.begin();

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

/**
 * Función placeDefenses por ordenación Rápida.
 * @param freeCells
 * @param nCellsWidth
 * @param nCellsHeight
 * @param mapWidth
 * @param mapHeight
 * @param obstacles
 * @param defenses
 */
void DEF_LIB_EXPORTED placeDefensesRapida(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
		, List<Object*> obstacles, List<Defense*> defenses)
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

    std::cout << "QuickSort: " << std::endl;
	for (int i = 0; i < nCellsWidth; ++i)
	{
		for (int j = 0; j < nCellsHeight; ++j)
		{
			v3tmp.x = (float)i;
			v3tmp.y = (float)j;
			VectorValues.emplace_back(cellsValues[i][j], v3tmp);
			std::cout << cellsValues[i][j] << "\t";
		}
		std::cout << std::endl;
	}

	/**
	 * Ordenacion de los valores de la lista.
	 */
	std::vector<ValueList> orderCells(VectorValues.begin(), VectorValues.end());
	//quickSort(orderCells, orderCells.size()-orderCells.size(), orderCells.size());
	VectorValues.sort(std::greater<ValueList>());

	std::list<ValueList>::iterator currentValue = VectorValues.begin();
    while (currentValue != VectorValues.end())
    {
        n++;
        std::cout << n << " - Place -- Valor celda: " << (*currentValue).value << ", Posicion celda: ["
                  << (*currentValue).position.x << ", " << (*currentValue).position.y << "]" << std::endl;
        ++currentValue;
    }
    currentValue = VectorValues.begin();
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
    std::cout << "eliminando cellsValues" << std::endl;
	for(int i = 0; i < nCellsHeight ; ++i)
		delete [] cellsValues[i];
	delete [] cellsValues;
	cellsValues = nullptr;
    std::cout << "fin orderRapida" << std::endl;
}

/**
 * Función placeDefenses por ordenación Monticulo.
 * @param freeCells
 * @param nCellsWidth
 * @param nCellsHeight
 * @param mapWidth
 * @param mapHeight
 * @param obstacles
 * @param defenses
 */
void DEF_LIB_EXPORTED placeDefensesMonticulo(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
		, List<Object*> obstacles, List<Defense*> defenses)
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
	std::vector<ValueList> orderCells(VectorValues.begin(), VectorValues.end());
    std::make_heap(orderCells.begin(), orderCells.end(), std::greater<ValueList>());//, std::greater<ValueList>()
    std::sort_heap(orderCells.begin(), orderCells.end(), std::greater<ValueList>());//, std::greater<ValueList>()
	//VectorValues.sort(std::greater<ValueList>());

	std::list<ValueList>::iterator currentValue = VectorValues.begin();

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

void DEF_LIB_EXPORTED placeDefenses3(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , List<Object*> obstacles, List<Defense*> defenses) {

	float cellWidth = mapWidth / nCellsWidth;
	float cellHeight = mapHeight / nCellsHeight;

	cronometro cNOrdenado;
	long int rNOrdenado = 0;
	cNOrdenado.activar();
	do {
		//codigo de placedefenses de la p1 sin ordenar, o llamar a la funcion
		placeDefensesNoOrdenacion(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
		++rNOrdenado;
	} while(cNOrdenado.tiempo() < e_abs / (e_rel + e_abs));
	cNOrdenado.parar();

	cronometro cFusion;
	long int rFusion = 0;
	cFusion.activar();
	do {
		//codigo de placedefenses de la p1 sin ordenar, o llamar a la funcion
		//placeDefensesFusion(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
		++rFusion;
	} while(cFusion.tiempo() < e_abs / (e_rel + e_abs));
	cFusion.parar();

	cronometro cRapida;
	long int rRapida = 0;
	cRapida.activar();
	do {
		//codigo de placedefenses de la p1 sin ordenar, o llamar a la funcion
		//placeDefensesRapida(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
		++rRapida;
	} while(cRapida.tiempo() < e_abs / (e_rel + e_abs));
	cRapida.parar();

	cronometro cMonticulo;
	long int rMonticulo = 0;
	cMonticulo.activar();
	do {
		//codigo de placedefenses de la p1 sin ordenar, o llamar a la funcion
		placeDefensesMonticulo(freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
		++rMonticulo;
	} while(cMonticulo.tiempo() < e_abs / (e_rel + e_abs));
	cMonticulo.parar();

	std::cout << nCellsWidth << "*" << nCellsHeight << '\t' << cNOrdenado.tiempo() / (double)rNOrdenado << '\t'
	<< cFusion.tiempo()*2 / (double)rFusion << '\t' << cRapida.tiempo()*3 / (double)rRapida << '\t'
	<< cMonticulo.tiempo()*4 / (double)rMonticulo << std::endl;
}
