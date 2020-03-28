/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*!
	\brief Wysokość podstawy.
	
	Wysokość podstawy.
*/
#define L1 	5		//Wysokosc podstawy
/*!
	\brief Długość pierwszego członu.
	
	Długość pierwszego członu.
*/
#define L2 	8.2		//Dlugosc pierwszego czlonu
/*!
	\brief Długość drugiego członu.
	
	Długość drugiego członu.
*/
#define L3 	7.7		//Dlugosc drugiego czlonu
/*!
	\brief Długość między przegubem, a osią serwomechanizmu od chwytaka.
	
	Długość między przegubem, a osią serwomechanizmu od chwytaka.
*/
#define L4 	4		//Dlugosc miedzy przegubem, a osia serwa od chwytaka
/*!
	\brief Obniżenie na chwytaku.
	
	Obniżenie na chwytaku.
*/
#define L5 	2.1		//Obnizenie na chwytaku
/*!
	\brief Długosc między osią serwomechanizmu chwytaka, a punktem chwytu.
	
	Długosc między osią serwomechanizmu chwytaka, a punktem chwytu.
*/
#define L6 	6		//Dlugosc miedzy osia serwa chwytaka, a punktem chwytu

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*!
 * \brief Tablica z kolejnymi maksymalnymi wartościami współrzędnej Z.
 *
 * Tablica z kolejnymi maksymalnymi wartościami współrzędnej Z, poczynając od r = 18, aż do r = 27, gdzie r jest długością ramienia
 * od środka podastawy do końca chwytaka.
 */
int16_t MaxymalneZ[] = {19, 19, 19, 18, 17, 16, 15, 13, 11, 9};

/*!
 * \brief Zmienna przechowująca odebrane dane.
 *
 * Zmienna przechowująca odebrane dane.
 */
char rx_data= '0';

/*!
 * \brief Tablica przechowująca dane do wysłania.
 *
 * Tablica przechowująca dane do wysłania.
 */
char tx_data[32];

/*!
 * \brief Flaga ustawiana przy odebraniu danych z USART-a.
 *
 * Flaga ustawiana przy odebraniu danych z USART-a.
 */
volatile uint8_t usart_rx;

/*!
 * \brief Flaga ustawiana po wysłaniu danych z USART-a.
 *
 * Flaga ustawiana po wysłaniu danych z USART-a.
 */
volatile uint8_t usart_tx;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*!
 *  \brief Funkcja odbierająca jeden znak z USART-a oraz wysyłająca ten sam znak z powrotem.
 *
 *  Funkcja pozwala odebrać jeden znak wysłany z komputera oraz wysłać z powrotem ten sam znak.
 *
 *  \return Znak odebrany od urządzenia zewnętrznego.
 */
char UartEcho(void)
{
	usart_rx= 0;
	HAL_UART_Receive_IT(&huart2,(unsigned char*)&rx_data,1);
	while(!usart_rx);
	usart_tx= 0;
	HAL_UART_Transmit_IT(&huart2,(unsigned char*)&rx_data,1);
	while(!usart_tx);
	return rx_data;
}

/*!
 *  \brief Funkcja pozwalająca wysłać ciąg znaków do zewnętrznego urządzenia (terminalu)
 *
 *  Funkcja pozwalająca wysłać ciąg znaków podanych jako parametr do zewnętrznego urządzenia (terminalu)
 *  \param[in] str - ciąg znaków, które chcemy wysłać.
 *
 *  \return Liczbę nieprzesłanych znaków.
 */
int UartPrint(char * str)
{
	int strsize= sprintf(tx_data,str);
	if(strsize > 0)
	{
		usart_tx= 0;
		HAL_UART_Transmit_IT(&huart2,(unsigned char*)tx_data,strsize);
		while(!usart_tx);
	}
	return strsize;
}

/*!
 *  \brief Funkcja pozwalająca wysłac liczbę całkowitą przez USART-a.
 *
 *  Funkcja pozwalająca wysłać liczbę całkowitą przez USART-a.
 *  \param[in] liczba - Liczba całkowita do wysłania
 *
 *  return Liczbę nieprzesłanych znaków.
 */
int UartPrintInt(int liczba)
{
	char str[12];
	int strsize= sprintf(str,"%d",liczba);
	if(strsize > 0)
		return UartPrint(str);
	return strsize;
}

/*!
 *  \brief Funkcja pozwalająca odebrać liczbę całkowitą.
 *
 *  Funkcja pozwalająca odebrać liczbę całkowitą.
 *  \param[in] num - wskaźnik na zmiennę, do której ma zostac zapisana odebrana liczba.
 *  \param[in] min - minimalna liczba jaką spodziewamy się odbierać.
 *  \param[in] max - maksymalna liczba jaką spodziewamy się odbierać.
 *
 *  \retval 1 - odebrano liczbę.
 *  \retval 0 - w odebranych danych był znak 'x' lub 'ESC'
 */
int UartReadInt( int * num, int min, int max )
{
	int suma= 0;
	char znak= '0';
	uint8_t minus = 0;
	while(znak != '\r' && znak != '\n' && znak != 'u')
	{
		znak= UartEcho();
		if(znak == 27 || znak == 'x')return 0;
		if(znak == '-') minus = 1;
		if(znak >= '0' && znak <= '9' && suma < max)
			suma= suma*10 + znak - '0';
	}
	*num= suma > min ? suma < max ? suma : max : min;
	if(minus == 1) *num = -(*num);
	return 1;
}

/*!
 *  \brief Funkcja obsługująca przerwanie po odebraniu danych.
 *
 *  Funkcja obsługująca przerwanie po odebraniu danych.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	usart_rx= 1;
}

/*!
 *  \brief Funkcja obsługująca przerwanie po wysłaniu danych.
 *
 *  Funkcja obsługująca przerwanie po wysłaniu danych.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	usart_tx= 1;
}


/*!
 *  \brief Redefinicja funkcji pozwalającej zatrzymać działanie programu na podany czas w milisekundach.
 *
 *  Redefinicja funkcji pozwalającej zatrzymać działanie programu na podany czas w milisekundach.
 *  \param[in] Delay - czas w milisekundach, na który należy wstrzymać działanie programu.
 */
__weak void HAL_Delay(__IO uint32_t Delay)

{

	uint32_t tickstart = 0;

	tickstart = HAL_GetTick();

	while ((HAL_GetTick() - tickstart) < Delay)

	{

	}

}

/*!
 *  \brief Funkcja pozwalająca podnieść pierwszy przegub o zadany kąt.
 *
 *   Funkcja pozwalająca podnieść pierwszy przegub o zadany kąt. Pozwala regulować prędkością tego ruchu, poprzez podzielenie
 *   kąta o jaki należy obrócic przegub na mniejszcze kąty oraz ustalic czas w milisekundach między kolejnymi obrotami o podane
 *   mniejsze kąty.
 *
 *   \param[in] alpha - kąt o jaki należy obrócic przegub.
 *   \param[in] del - przerwa w milisekundach, między kolejnymi obrotami o mniejsze kąty.
 *   \param[in] skok - wartość o jaka będzie się zmieniac wartość w rejestrze pozwalająca określić jak małe będą kąty pośrednie.
 *
 *   \retval 0 - zwraca kiedy funkcja się zakończy.
 */
uint8_t moveAlpha(uint16_t alpha, uint16_t del, uint16_t skok) {
	int pwm = 0;
	int i = 0;

	if(TIM1->CCR4 == 0)
	{
		for (i = 970; i < 1206 - skok; i += skok)
		{
			TIM1->CCR4 = i;

			HAL_Delay(del);
		}
	}

	if (alpha >= 70 && alpha <= 105)
	{
		pwm = (int) (alpha * 5.1); // przelicznik kąta na skale pwm
		pwm += 670;
		i = TIM1->CCR4;

		// punkt poczatkowy (90stopni) to 670
	} else if (alpha >= 0 && alpha <= 70)
	{
		pwm = (int) (alpha * 5.3); // przelicznik kąta na skale pwm
		pwm += 670;


	} else {
		return 1;
	}

	i = TIM1->CCR4;
	if (i < pwm) {
		for (; i < pwm - skok; i += skok) {

			TIM1->CCR4 = i;

			HAL_Delay(del);

		}

	} else if (i > pwm) {
		for (; i > pwm + skok; i -= skok) {

			TIM1->CCR4 = i;

			HAL_Delay(del);
		}
	}
	TIM1->CCR4 = pwm;
	return 0;

}

/*!
 *  \brief Funkcja pozwalająca zmieniać kąt między pierwszym i drugim przegubem.
 *
 *   Funkcja pozwalająca zmieniać kąt między pierwszym i drugim przegubem. Pozwala regulować prędkością tego ruchu, poprzez podzielenie
 *   kąta o jaki należy obrócic przegub na mniejszcze kąty oraz ustalic czas w milisekundach między kolejnymi obrotami o podane
 *   mniejsze kąty.
 *
 *   \param[in] beta - kąt o jaki należy obrócic przegub.
 *   \param[in] del - przerwa w milisekundach, między kolejnymi obrotami o mniejsze kąty.
 *   \param[in] skok - wartość o jaka będzie się zmieniac wartość w rejestrze pozwalająca określić jak małe będą kąty pośrednie.
 *
 *   \retval 0 - zwraca kiedy funkcja się zakończy.
 */
 */
uint8_t moveBeta(uint16_t beta, uint16_t del, uint16_t skok) {
	int pwm = 0;
	int i = 0;

	if(TIM1->CCR3 == 0)
	{
		for (i = 670; i < 1097 - skok; i += skok)
		{
			TIM1->CCR3 = i;

			HAL_Delay(del);
		}
	}

	if (beta > 68 && beta < 160) {
		pwm = (int) ((beta - 68) * 6.1); // przelicznik kąta na skale pwm

	} else if (beta >= 160 && beta < 250) {
		pwm = (int) ((beta - 68) * 5.4); // przelicznik kąta na skale pwm

	} else {
		return 1;
	}

	pwm = 1250 - pwm; // punkt poczatkowy (90stopni) to 670

	// punkt poczatkowy (90stopni) to 670
	i = TIM1->CCR3;
	if (i < pwm) {
		for (; i < pwm; i += skok) {

			TIM1->CCR3 = i;

			HAL_Delay(del);

		}

	} else if (i > pwm) {
		for (; i > pwm; i -= skok) {

			TIM1->CCR3 = i;

			HAL_Delay(del);
		}
	}

	TIM1->CCR3 = pwm;
	return 0;

}

/*!
 *  \brief Funkcja pozwalająca zmieniać kąt między drugim przegubem, a chwytakiem.
 *
 *   Funkcja pozwalająca zmieniać kąt między drugim przegubem, a chwytakiem. Pozwala regulować prędkością tego ruchu, poprzez podzielenie
 *   kąta o jaki należy obrócic przegub na mniejszcze kąty oraz ustalic czas w milisekundach między kolejnymi obrotami o podane
 *   mniejsze kąty.
 *
 *   \param[in] gamma - kąt o jaki należy obrócic przegub.
 *   \param[in] del - przerwa w milisekundach, między kolejnymi obrotami o mniejsze kąty.
 *   \param[in] skok - wartość o jaka będzie się zmieniac wartość w rejestrze pozwalająca określić jak małe będą kąty pośrednie.
 *
 *   \retval 0 - zwraca kiedy funkcja się zakończy.
 */
 
uint8_t moveGamma(uint16_t gamma, uint16_t del, uint16_t skok) {

	int pwm = 0;
	int i = 0;

	if(TIM1->CCR1 == 0)
	{
		for (i = 670; i < 1036 - skok; i += skok)
		{
			TIM1->CCR1 = i;

			HAL_Delay(del);
		}
	}

	if (gamma > 110 && gamma < 250) {
		pwm = (int) ((gamma - 110) * 4.5); // przelicznik kąta na skale pwm

	} else if (gamma >= 250 && gamma < 294) {
		pwm = (int) ((gamma - 110) * 4.85); // przelicznik kąta na skale pwm

	} else {
		return 1;
	}

	pwm += 250; // punkt poczatkowy (90stopni) to 670

	// punkt poczatkowy (90stopni) to 670
	i = TIM1->CCR1;
	if (i < pwm) {
		for (; i < pwm - skok; i += skok) {

			TIM1->CCR1 = i;

			HAL_Delay(del);

		}

	} else if (i > pwm) {
		for (; i > pwm + skok; i -= skok) {

			TIM1->CCR1 = i;

			HAL_Delay(del);
		}
	}

	TIM1->CCR1 = pwm;
	return 0;

}

/*!
 *  \brief Funkcja pozwalająca zmieniać kąt obrotu manipulatora.
 *
 *  Funkcja pozwalająca zmieniać kąt obrotu manipulatora. Pozwala regulować prędkością tego ruchu, poprzez podzielenie
 *   kąta o jaki należy obrócic przegub na mniejszcze kąty oraz ustalic czas w milisekundach między kolejnymi obrotami o podane
 *   mniejsze kąty.
 *
 *   \param[in] zero - kąt o jaki należy obrócic przegub.
 *   \param[in] del - przerwa w milisekundach, między kolejnymi obrotami o mniejsze kąty.
 *   \param[in] skok - wartość o jaka będzie się zmieniac wartość w rejestrze pozwalająca określić jak małe będą kąty pośrednie.
 *
 *   \retval 0 - zwraca kiedy funkcja się zakończy.
 */
uint8_t moveZero(uint16_t zero, uint16_t del, uint16_t skok) {
	int i = 0;
	int pwm = 0;

	if(TIM1->CCR2 == 0)
	{
		for (i=670; i < 725 - skok; i += skok)
		{
			TIM1->CCR2 = i;

			HAL_Delay(del);
		}
	}

	if (zero > 0 && zero < 100) {
		pwm = (int) (zero * 5.05);

	} else if (zero >= 100 && zero < 175) {
		pwm = (int) (zero * 5.6);

	} else {
		return 1;
	}

	pwm += 270;

	// punkt poczatkowy (90stopni) to 670
	i = TIM1->CCR2;
	if (i < pwm) {
		for (; i < pwm - skok; i += skok) {

			TIM1->CCR2 = i;

			HAL_Delay(del);

		}

	} else if (i > pwm) {
		for (; i > pwm + skok; i -= skok) {

			TIM1->CCR2 = i;

			HAL_Delay(del);
		}
	}

	TIM1->CCR2 = pwm;
	return 0;

}

/*!
 * \brief Funkcja pozwalająca zamknąć chwytak.
 *
 *  Funkcja pozwalająca zamknąć chwytak.
 */
void closeGrip(){
	if(TIM3->CCR4!=1160){
	for(int i = 1000;  i<1160; i+=5)
	{TIM3->CCR4=i;}}
	else return;

}

/*!
 * \brief Funkcja pozwalająca otworzyć chwytak.
 *
 *  Funkcja pozwalająca otworzyć chwytak.
 */
void openGrip(){
	if(TIM3->CCR4!=1000){
		for(int i = 1160;  i>1000; i-=5)
	{TIM3->CCR4=i;}}
	else return ;


}

/*!
 *  \brief Funkcja ustawiająca manipulator w pozycji początkowej.
 *
 *  Funkcja ustawiająca manipulator w pozycji początkowy.
 */
uint8_t PozycjaPoczatkowa() {
	moveAlpha(100, 10, 5);
	moveGamma(250, 10, 5);
	moveBeta(90, 10, 5);
	moveZero(90, 10, 5);
	openGrip();

	return 0;
}

/*!
 * 	\brief Funkcja ustawiająca manipulator w zadanym punkcie przestrzeni zadaniowej.
 *
 * 	Funkcja ustawiająca manipulator w zadanym punkcie przestrzeni zadaniowej. Wykorzystuje ona do obliczenia odpowiednich kątów
 * 	kinematykę odwrotną podaną w postaci jawnej. Po obliczeniu kątów ustawia je powodując ruch manipulatora.
 *
 * 	\param[in] X - współrzędna X w przestrzeni zadaniowej.
 * 	\param[in] Y - współrzędna Y w przestrzeni zadaniowej.
 * 	\param[in] Z - współrzędna Z w przestrzeni zadaniowej.
 *
 * 	\retval 0 - kiedy funkcja zakończy się poprawnie.
 * 	\retval 1 - kiedy podane współrzędne wykraczają poza przestrzeń zadaniową. Próbujemy obrócic manipulator za daleko.
 * 	\retval 2 - kiedy podane współrzędne wykraczają poza przestrzeń zadaniową. Próbujemy sięgnąc za daleko lub za blisko.
 * 	\retval 3 - kiedy podane współrzędne wykraczają poza przestrzeń zadaniową. Próbujemy ustaic ostatni człon manipulatora za wysoko.
 *
 */
uint8_t UstawManipulator(int16_t X, int16_t Y, int16_t Z) {


	//Jezeli x sa male to nie mozna dac ujemnych y (kat zero do 175 stopni)
	if(X <= 1 && Y < 0)
	{
		return 1;
	}

	//Sprawdza czy wspolrzedne mieszcza sie w zasiegu ramienia
	if(pow(X,2) + pow(Y,2) > 729 || pow(X,2) + pow(Y,2) < 314)
	{
		return 2;
	}

	int zasieg = ceil(sqrt(pow(X,2) + pow(Y,2)));

	//Sprawdza czy nie probujemy podnies chwytaka zbyt wysoko lub nisko
	if(Z > MaxymalneZ[zasieg-18] || Z < 1)
	{
		return 3;
	}

	UartPrintInt(Y);
	float alpha = 0;
	float beta = 0;
	float gamma = 0;
	float zero = 0;
	float pom1 = 0;
	float pom2 = 0;

	Z -= L1;

	float r = sqrt(pow(X, 2) + pow(Y, 2));
	float r1 = r - L4 - L6;
	float a = sqrt(pow(r1, 2) + pow(Z, 2));

	beta = acos(
			(float) (pow(L2, 2) + pow(L3, 2) - pow(a, 2))
					/ (float) (2 * L2 * L3));
	pom1 = asin((float) Z / a);
	pom2 = asin((sin(beta) * L2) / a);
	alpha = pom1 + pom2;
	gamma = 2 * M_PI - beta - alpha;

	zero = acos((float) Y / r);

	int kat0 = round(zero * 180 / M_PI);
	int kat1 = round(alpha * 180 / M_PI);
	int kat2 = round(beta * 180 / M_PI);
	int kat3 = round(gamma * 180 / M_PI);

	UartPrintInt(0);
	moveZero(kat0, 20, 5);
	moveAlpha(kat1, 20, 5);
	moveBeta(kat2, 20, 5);
	moveGamma(kat3, 20, 5);



	return 0;

}


/*!
 * \brief Funkcja pozwalająca przemieścić chwytak po lini prostej (możliwie dokładnie).
 *
 * Funkcja pozwalająca przemieścić chwytak po linii prostej (możliwie dokładnie).
 *
 * \param[in] X1 - współrzędna X w przestrzeni zadaniowej, w której znajduje się chwytak na początku ruchu.
 * \param[in] Y1 - współrzędna Y w przestrzeni zadaniowej, w której znajduje się chwytak na początku ruchu.
 * \param[in] Z1 - współrzędna Z w przestrzeni zadaniowej, w której znajduje się chwytak na początku ruchu.
 * \param[in] X2 - współrzędna X w przestrzeni zadaniowej, do której chcemy przemieścic chwytak.
 * \param[in] Y2 - współrzędna Y w przestrzeni zadaniowej, do której chcemy przemieścic chwytak.
 * \param[in] Z2 - współrzędna Z w przestrzeni zadaniowej, do której chcemy przemieścic chwytak.
 * \param[in] Dokladnosc - ilość punktów pośrednich między pozycją początkową, a końcową (maksymalnie 20).
 *
 * \retval 0 - kiedy chwytak został przesunięty.
 * \retval 1 - kiedy chwytak nie został przesunięty (pozycja początkowa i końcowa jest identyczna).
 *
 */
uint8_t MoveL(int16_t X1, int16_t Y1, int16_t Z1, int16_t X2, int16_t Y2, int16_t Z2, uint8_t Dokladnosc)
{
	if(Dokladnosc > 20 || Dokladnosc == 0) Dokladnosc = 20;


	int16_t OdlegloscX = abs(X1-X2);
	int16_t OdlegloscY = abs(Y1-Y2);
	int16_t OdlegloscZ = abs(Z1-Z2);
	float Podzial = 1;
	int16_t x, y, z;


	if (OdlegloscX > 0)
	{
		if (OdlegloscX >= OdlegloscY && OdlegloscX >= OdlegloscZ)
		{
			if (OdlegloscX >= Dokladnosc)
				Podzial = (float)(OdlegloscX/(float)Dokladnosc);
			else
				Podzial = 1;

			if (OdlegloscY > 0 && OdlegloscZ > 0)
			{
				x = X1;

				while ((X1 > X2 && floor(x - Podzial) > X2) || (X1 < X2 && floor(x + Podzial) < X2))
				{
	                    if( X1 > X2)
	                        x = floor(x - Podzial);
	                    else
	                        x = floor(x + Podzial);

	                    y = floor(((x - X1)*(Y2 - Y1)/(X2 - X1)) + Y1);
	                    z = floor(((x - X1)*(Z2 - Z1)/(X2 - X1)) + Z1);

	                    UstawManipulator(x,y,z);
				}

				UstawManipulator(X2,Y2,Z2);

			}
			else if (OdlegloscY > 0 && OdlegloscZ <= 0)
			{
				x = X1;

				while ((X1 > X2 && floor(x - Podzial) > X2) || (X1 < X2 && floor(x + Podzial) < X2))
				{
					if (X1 > X2)
						x = floor(x - Podzial);
					else
						x = floor(x + Podzial);

					y = floor(((x - X1) * (Y2 - Y1) / (X2 - X1)) + Y1);
					z = Z1;

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);
			}
			else if( OdlegloscZ > 0 && OdlegloscY <= 0)
			{
				x = X1;

				while ((X1 > X2 && floor(x - Podzial) > X2) || (X1 < X2 && floor(x + Podzial) < X2))
				{
					if (X1 > X2)
						x = floor(x - Podzial);
					else
						x = floor(x + Podzial);

					y = Y1;
					z = floor(((x - X1)*(Z2 - Z1)/(X2 - X1)) + Z1);

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);

			}
			else if (OdlegloscZ <= 0 && OdlegloscY <= 0)
			{
				x = X1;

				while((X1 > X2 && floor(x - Podzial) > X2) || (X1 < X2 && floor(x + Podzial) < X2))
				{
					if (X1 > X2)
						x = floor(x - Podzial);
					else
						x = floor(x + Podzial);

					y = Y1;
					z = Z1;

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);
			}

			return 0;
		}
	}


	if (OdlegloscY > 0)
	{
		if (OdlegloscY >= OdlegloscX && OdlegloscY >= OdlegloscZ)
		{
			if (OdlegloscY >= Dokladnosc)
				Podzial = (float)(OdlegloscY / (float)Dokladnosc);
			else
				Podzial = 1;

			if (OdlegloscX > 0 && OdlegloscZ > 0)
			{
				y = Y1;

				while ((Y1 > Y2 && floor(y - Podzial) > Y2) || (Y1 < Y2 && floor(y + Podzial) < Y2))
				{
					if (Y1 > Y2)
						y = floor(y - Podzial);
					else
						y = floor(y + Podzial);

					x = floor(((y - Y1) * (X2 - X1) / (Y2 - Y1)) + X1);
					z = floor(((y - Y1) * (Z2 - Z1) / (Y2 - Y1)) + Z1);

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);

			}
			else if (OdlegloscX > 0 && OdlegloscZ <= 0)
			{
				y = Y1;

				while ((Y1 > Y2 && floor(y - Podzial) > Y2) || (Y1 < Y2 && floor(y + Podzial) < Y2))
				{
					if (Y1 > Y2)
						y = floor(y - Podzial);
					else
						y = floor(y + Podzial);

					x = floor(((y - Y1) * (X2 - X1) / (Y2 - Y1)) + X1);
					z = Z1;

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);

			}
			else if (OdlegloscZ > 0 && OdlegloscX <= 0)
			{
				y = Y1;

				while ((Y1 > Y2 && floor(y - Podzial) > Y2) || (Y1 < Y2 && floor(y + Podzial) < Y2))
				{
					if (Y1 > Y2)
						y = floor(y - Podzial);
					else
						y = floor(y + Podzial);

					x = X1;
					z = floor(((y - Y1) * (Z2 - Z1) / (Y2 - Y1)) + Z1);

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);
			}
			else if (OdlegloscZ <= 0 && OdlegloscX <= 0)
			{
				y = Y1;

				while ((Y1 > Y2 && floor(y - Podzial) > Y2) || (Y1 < Y2 && floor(y + Podzial) < Y2))
				{
					if(Y1 > Y2)
						y = floor(y - Podzial);
					else
						y = floor(y + Podzial);

					x = X1;
					z = Z1;


					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);
			}

			return 0;
		}

	}

	if (OdlegloscZ > 0)
	{
		if (OdlegloscZ >= OdlegloscX && OdlegloscZ >= OdlegloscY)
		{
			if (OdlegloscZ >= Dokladnosc)
				Podzial = (float)(OdlegloscZ / (float)Dokladnosc);
			else
				Podzial = 1;

			if (OdlegloscX > 0 && OdlegloscY > 0)
			{
				z = Z1;

				while ((Z1 > Z2 && floor(z - Podzial) > Z2) || (Z1 < Z2 && floor(z + Podzial) < Z2))
				{
					if (Z1 > Z2)
						z = floor(z - Podzial);
					else
						z = floor(z + Podzial);

					x = floor(((z - Z1) * (X2 - X1) / (Z2 - Z1)) + X1);
					y = floor(((z - Z1) * (Y2 - Y1) / (Z2 - Z1)) + Y1);

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);

			}
			else if (OdlegloscX > 0 && OdlegloscY <= 0)
			{
				z = Z1;

				while ((Z1 > Z2 && floor(z - Podzial) > Z2) || (Z1 < Z2 && floor(z + Podzial) < Z2))
				{
					if (Z1 > Z2)
						z = floor(z - Podzial);
					else
						z = floor(z + Podzial);

					x = floor(((z - Z1) * (X2 - X1) / (Z2 - Z1)) + X1);
					y = Y1;

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);

			}
			else if (OdlegloscY > 0 && OdlegloscX <= 0)
			{
				z = Z1;

				while ((Z1 > Z2 && floor(z - Podzial) > Z2) || (Z1 < Z2 && floor(z + Podzial) < Z2))
				{
					if (Z1 > Z2)
						z = floor(z - Podzial);
					else
						z = floor(z + Podzial);

					x = X1;
					y = floor(((z - Z1) * (Y2 - Y1) / (Z2 - Z1)) + Y1);

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);

			}
			else if (OdlegloscY <= 0 && OdlegloscX <= 0)
			{
				z = Z1;

				while ((Z1 > Z2 && floor(z - Podzial) > Z2) || (Z1 < Z2 && floor(z + Podzial) < Z2))
				{
					if (Z1 > Z2)
						z = floor(z - Podzial);
					else
						z = floor(z + Podzial);

					x = X1;
					y = Y1;

					UstawManipulator(x, y, z);
				}

				UstawManipulator(X2, Y2, Z2);
			}

			return 0;
		}
	}


	return 1;

}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //HAL_UART_RxCpltCallback(&huart1);
//PozycjaPoczatkowa();
//UstawManipulator(20,0,3);
//HAL_Delay(10000);
//closeGrip();
//HAL_Delay(10000);
//UstawManipulator(20,0,6);
//HAL_Delay(5000);
//UstawManipulator(17,10,6);
//HAL_Delay(10000);
//UstawManipulator(17,10,3);
//HAL_Delay(3000);
//openGrip();
//HAL_Delay(1000);
//PozycjaPoczatkowa();


	int X1 = 0, Y1 = 0, X2 = 0, Y2 = 0;
	uint8_t PoprawneWspolrzedne = 0;


	UartPrint("Witamy w interfejsie manipulatora\r\n");
	UartPrint("\r\nMenu: \r\n");
	UartPrint("p - Przenies pionek\r\n");
	UartPrint("m - menu\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	  if(rx_data != '\0')
	  {
			switch(rx_data)
			{
				case 'p':

					PoprawneWspolrzedne = 0;

					while(PoprawneWspolrzedne == 0)
					{
						UartPrint("Podaj obecna wspolrzedna X pionka: \r\n");
						UartReadInt(&X1,0,27);
						UartPrint("\r\nPodaj obecna wspolrzedna Y pionka: \r\n");
						UartReadInt(&Y1,0,27);


						if(X1 <= 1 && Y1 < 0)
						{
							UartPrint("Podane wspolrzedne sa nieprawidlowe, wychodza poza przestrzen zadaniowa, dla X <= 1, Y musi byc dodatnie ! \r\n");
							UartPrint("Sprobuje jeszcze raz\r\n");
							continue;
						}


						if(pow(X1,2) + pow(Y1,2) > 729 || pow(X1,2) + pow(Y1,2) < 314)
						{
							UartPrint("Podane wspolrzedne sa nieprawidlowe, wychodza poza przestrzen zadaniowa, dla X <= 1, Y musi byc dodatnie ! \r\n");
							UartPrint("Sprobuje jeszcze raz\r\n");
							continue;
						}

						PoprawneWspolrzedne = 1;

					}


					PoprawneWspolrzedne = 0;

					while(PoprawneWspolrzedne == 0)
					{
						UartPrint("\r\nPodaj wspolrzedna X, na ktora ustawic pionka: \r\n");
						UartReadInt(&X2,0,27);
						UartPrint("\r\nPodaj wspolrzedna Y, na ktora ustawic pionka: \r\n");
						UartReadInt(&Y2,0,27);

						if(X2 <= 1 && Y2 < 0)
						{
							UartPrint("Podane wspolrzedne sa nieprawidlowe, wychodza poza przestrzen zadaniowa, dla X <= 1, Y musi byc dodatnie ! \r\n");
							UartPrint("Sprobuje jeszcze raz\r\n");
							continue;
						}


						if(pow(X2,2) + pow(Y2,2) > 729 || pow(X2,2) + pow(Y2,2) < 314)
						{
							UartPrint("Podane wspolrzedne sa nieprawidlowe, wychodza poza przestrzen zadaniowa, dla X <= 1, Y musi byc dodatnie ! \r\n");
							UartPrint("Sprobuje jeszcze raz\r\n");
							continue;
						}

						PoprawneWspolrzedne = 1;
					}

					PozycjaPoczatkowa();
					HAL_Delay(2000);
					UstawManipulator(X1,Y1,3);
					HAL_Delay(3000);
					closeGrip();
					HAL_Delay(3000);
					MoveL(X1,Y1,3,X1,Y1,6,20);
					HAL_Delay(3000);
					MoveL(X1,Y1,6,X2,Y2,6,20);
					HAL_Delay(5000);
					MoveL(X2,Y2,6,X2,Y2,3,20);
					HAL_Delay(3000);
					openGrip();
					HAL_Delay(1000);
					PozycjaPoczatkowa();
					HAL_Delay(2000);

					UartPrint("\r\nGotowe\r\n");
					UartPrint("Podaj nowa komende\r\n");
					break;
					
				case 'm':
					UartPrint("\r\nMenu: \r\n");
					UartPrint("p - Przenies pionek\r\n");
					UartPrint("m - menu\r\n");
					
					break;

				default:
					UartPrint("Odczytano kod: ");
					UartPrintInt((int)rx_data);
					UartPrint("\r\n");
			}
	  }

	  rx_data= '\0';

	  HAL_UART_Receive_IT(&huart2,(unsigned char*)&rx_data,1);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
