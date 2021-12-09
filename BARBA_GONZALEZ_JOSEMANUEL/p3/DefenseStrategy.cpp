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
#endif

void orderFusion(std::vector<float>& orderCells, size_t pos_i, size_t pos_j)
{
	size_t n = pos_j - pos_i + 1;
	size_t n0 = 3, pos_k;

	if (n <= n0)
	{
		float maxim = std::max(orderCells[pos_i], std::max(orderCells[pos_i +1],  orderCells[pos_j]));
		float minim = std::min(orderCells[pos_i], std::min(orderCells[pos_i +1],  orderCells[pos_j]));
		float inter;
		if(orderCells[pos_i] < maxim and orderCells[pos_i] >= minim)
			inter = orderCells[pos_i];
		else if(orderCells[pos_i + 1] < maxim and orderCells[pos_i + 1] >= minim)
			inter = orderCells[pos_i + 1];
		else
			inter = orderCells[pos_j];
		orderCells[pos_i] = maxim; orderCells[pos_i + 1] = inter; orderCells[pos_j] = minim;
	}
	else
	{
		pos_k = (pos_i - 1) + (n / 2);
		orderFusion(orderCells, pos_i, pos_k);
		orderFusion(orderCells, pos_k + 1, pos_j);
	}
}

void orderRapida()
{}

void orderMonticulo()
{}

void DEF_LIB_EXPORTED placeDefensesNoOrdenacion(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
		, List<Object*> obstacles, List<Defense*> defenses) {

}

void DEF_LIB_EXPORTED placeDefensesFusion(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
		, List<Object*> obstacles, List<Defense*> defenses) {

}

void DEF_LIB_EXPORTED placeDefensesRapida(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
		, List<Object*> obstacles, List<Defense*> defenses) {

}

void DEF_LIB_EXPORTED placeDefensesMonticulo(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
		, List<Object*> obstacles, List<Defense*> defenses) {

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
		++rNOrdenado;
	} while(cNOrdenado.tiempo() < e_abs / (e_rel + e_abs));
	cNOrdenado.parar();

	cronometro cFusion;
	long int rFusion = 0;
	cFusion.activar();
	do {

		//codigo de placedefenses de la p1 sin ordenar, o llamar a la funcion
		++rFusion;
	} while(cFusion.tiempo() < e_abs / (e_rel + e_abs));
	cFusion.parar();

	cronometro cRapida;
	long int rRapida = 0;
	cRapida.activar();
	do {

		//codigo de placedefenses de la p1 sin ordenar, o llamar a la funcion
		++rRapida;
	} while(cRapida.tiempo() < e_abs / (e_rel + e_abs));
	cRapida.parar();

	cronometro cMonticulo;
	long int rMonticulo = 0;
	cMonticulo.activar();
	do {

		//codigo de placedefenses de la p1 sin ordenar, o llamar a la funcion
		++rMonticulo;
	} while(cMonticulo.tiempo() < e_abs / (e_rel + e_abs));
	cMonticulo.parar();

	std::cout << (nCellsWidth * nCellsHeight) << '\t' << c.tiempo() / r << '\t' << c.tiempo()*2 / r << '\t' << c.tiempo()*3 / r << '\t' << c.tiempo()*4 / r << std::endl;
}
