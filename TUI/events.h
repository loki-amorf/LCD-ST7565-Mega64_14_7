/**
 * \file events.h
 *
 */

#ifndef EVENTS_H_
#define EVENTS_H_
#include "buttons.h" 

/// ����������� ���� �������
typedef enum{
	EV_NONE = 0,	/// ��� �������
	EV_PREV = 1,	/// ����������
	EV_NEXT = 2,	/// ���������
	EV_ENTER = 3,	/// ����, �����
	EV_ESCAPE = 4	/// ������, �����
} EVENT;

/** ������ �������� �������.
 * ������� ��������� ����������� ��������� ��������� �������. ����������
 * �������� ������� (����������, �������, �������� � �.�.) ������������ 
 * ������������� � ����������� �� ������ �������.
 * @return ������� �������
 */
#define get_event() get_key()


#endif /* EVENTS_H_ */
