/** \file buttons.h
 * \brief ������������ ���� ������ ��������� ������
 * �������� ��������� � �������, ������������ � ���������
 * \par
 * \author ARV
 * \note
 * \n �����:
 * \n \date	14.11.2008 ... 	__.__.2008
 * \par
 * \version <������>.	\n
 * Copyright 2008 � ARV. All rights reserved. </b>
 * \par
 * ��� ������ ������� ���������:\n
 * 	-# WinAVR-20080411 ��� ����� ����� ������
 *
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

/// ����� �������� ����������� �� ������� ������ ��������� �� �����������
#define FIRST_RPT 100
/// ����� �������� ����������� �� ����������� ������� ������
#define RPT_DELAY 5

#define KEYPIN		PD5

/// ��� ���������� ������� ������
#define k_none		0
/// ��� ������ SET
#define k_set		_BV(0)
/// ��� ������ �����
#define k_up		_BV(1)
/// ��� ������ MODE
#define k_mode		_BV(2)
/// ��� ������ ����
#define k_dn 		_BV(3)

/// ������� �������� ���� ������� ������
uint8_t get_key(void);

#endif /* BUTTONS_H_ */
