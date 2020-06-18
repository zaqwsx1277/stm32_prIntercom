/*
 * TComDef.hpp
 *
 *  Created on: May 8, 2020
 *      Author: AAL
 */

#ifndef INC_TCOMDEF_HPP_
#define INC_TCOMDEF_HPP_


#include "speex/speex.h"

//#include "speex/speex_echo.h"

#define defVoiceInputBufSize 160	///< Размер входного буфера для кодека speex
#define defVoiceEncodeBufSize 20	///< Размер сжатого буфера speex
#define defNumBuf 2 				///< Кол-во обрабатываемых буферов (почему из может быть больше дыух не знаю)
#define defStartPeriod 8000			///< Длительность начального периода для вычисления среднего значения уровня входного сигнала.
//#define defTim6CountPeriod 7199		///< Счётчик для таймера ожидания прекращения передачи голоса (Таймер программируется на ожидание 1 сек)

#define defColorLight1 GPIO_PIN_1	// Настройка на цвета светодиодов. Нужны для того, что бы у контроллеров отличались основные цвета светодиодов
#define defColorLight2 GPIO_PIN_2

#define startUART_DMA HAL_UART_Receive_DMA(&huart3, (uint8_t *) &stVoiceOut, 2) ///< Макрос для инициализации приёма данных по DMA


	typedef enum protocol {trAsk, trConfirm, trReject,trSendData, trReceiveData, trFinish} defProtocol ; ///< Протокол взаимодействия контроллеров.

	typedef enum protocolErr {errNo, errUnknown} defProtocolErr ;
	enum trState {stateUnknown,
				  stateWait, 			// Ожидание передачи или приёма
				  stateStart, 			// Начальный период
				  stateReady, 			// Нажата кнопка передачи звука
				  stateTrRead,
				  stateTrSend,
				  stateVoicePlay, 		// Воспроизведения звука
				  stateVoiceReceive,	// Получены 2 байта звка
				  stateError} ;

//	struct defProtocol {
//		defProtocol command;
//		uint16_t *ptrData ;
//	} ;

	volatile static uint16_t stState = stateStart ;				///< Текущее состояние контроллера
	volatile static uint32_t stStartPeriod = 0 ; 				///< Счётчик тиков начального периода
//	volatile static uint64_t stVoiceEncodeAverage = 0 ;			///< Среднее значение шума нужное для адекватной работы ЦАП

//	volatile static int16_t stVoiceEncodeBuf [defNumBuf][defVoiceInputBufSize] ;	///< Буфер для записи данных с микрофона.
//	volatile static int16_t stVoiceDecodeBuf [defNumBuf][defVoiceInputBufSize] ;	///< Буфер для записи данных для воспроизведения.
//	volatile static uint16_t stSpeexEncodeBuf [defVoiceEncodeBufSize] ;		///< Буфер для сжатых исходных данных
//	volatile static uint16_t stSpeexDecodeBuf [defVoiceEncodeBufSize] ;		///< Буфер для сжатых полученных данных
//	volatile static uint16_t stVoiceEncodeBufPos = 0 ;					///< Текущая позиция в буфере stVoiceEncodeBuf
//	volatile static uint16_t stVoiceEncodeBufNum = 0 ;					///< Текущий обрабатываемый буфер для исходных данных
//	volatile static uint32_t stVoiceEncodeErr = 0 ;						///< Кол-во ошибок заполнения входного буфера
//	volatile static uint16_t stVoiceDecodeBufPos = 0 ;					///< Текущая позиция в буфере stVoiceDecodeBuf
//	volatile static uint16_t stVoiceDecodeBufNum = 0 ;					///< Текущий обрабатываемый буфер для декодированных данных
//	volatile static uint32_t stVoiceDecodeErr = 0 ;						///< Кол-во ошибок воспроизведения декодированных данных
//	volatile static uint16_t stVoiceEncodeAverage = 2000 ;				///< Среднее значение оцифрованного шума. Нужно для корректной работы декодирования speex

//	void *stSpeexEncodeHandle, *stSpeexDecodeHandle ;					///< Указатель на структуру описывающую состояние кодека
//	static SpeexBits stSpeexEncodeStream, stSpeexDecodeStream ;			///< Структура для работы с кодеком speex
////	static SpeexEchoState *stSpeexEcho ;								///< Структура для работы с подавлением эха
//	volatile static int32_t stSpeexQuality = 4 ;						///< Качество. От него зависит используемый битрейт. 0 - загрузка минимальная, 10 максимальная.
//	volatile static int32_t stSpeexComplexity = 2 ;						///< Установка загрузки процессора
//	volatile static int32_t stSpeexVBR = 1 ;							///< Флаг установки переменного битрейта 0- выключен
//	volatile static int32_t stSpeexEnh = 1 ;							///< Флаг регулировки усиления

//	static uint32_t stTestClock = {0} ;
//	static uint64_t stTestVal = 1 ;
	static uint32_t stVoiceIn = 0 ;										///< Данные получаемые с АЦП
	volatile static uint32_t stVoiceOut = 0 ;							///< Данные подготовленные для вывода в ЦАП

//	volatile static uint32_t stPeriodVoiceReceive = 0 ;					///< Счётчик ожидания окончания передачи голоса

	void managerState () ;												///< Менеджер обработки состояний
	void managerTransfer () ;											///< Менеджер сжатия кодеком и передачи данных

#endif /* INC_TCOMDEF_HPP_ */
