// Physics.cpp: 응용 프로그램의 진입점을 정의합니다.
//

#include "stdafx.h"
#include "Physics.h"
#include <stdio.h>

#define MAX_LOADSTRING 100

// 전역 변수:
HINSTANCE hInst;                                // 현재 인스턴스입니다.
WCHAR szTitle[MAX_LOADSTRING];                  // 제목 표시줄 텍스트입니다.
WCHAR szWindowClass[MAX_LOADSTRING];            // 기본 창 클래스 이름입니다.

// 이 코드 모듈에 들어 있는 함수의 정방향 선언입니다.
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// TODO: 여기에 코드를 입력합니다.

	// 전역 문자열을 초기화합니다.
	LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadStringW(hInstance, IDC_PHYSICS, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// 응용 프로그램 초기화를 수행합니다.
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_PHYSICS));

	MSG msg;

	// 기본 메시지 루프입니다.
	while (GetMessage(&msg, nullptr, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int)msg.wParam;
}



//
//  함수: MyRegisterClass()
//
//  목적: 창 클래스를 등록합니다.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEXW wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_PHYSICS));
	wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_PHYSICS);
	wcex.lpszClassName = szWindowClass;
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassExW(&wcex);
}

//
//   함수: InitInstance(HINSTANCE, int)
//
//   목적: 인스턴스 핸들을 저장하고 주 창을 만듭니다.
//
//   설명:
//
//        이 함수를 통해 인스턴스 핸들을 전역 변수에 저장하고
//        주 프로그램 창을 만든 다음 표시합니다.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance; // 인스턴스 핸들을 전역 변수에 저장합니다.

	HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

	if (!hWnd)
	{
		return FALSE;
	}

	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	return TRUE;
}

//
//  함수: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  목적:  주 창의 메시지를 처리합니다.
//
//  WM_COMMAND  - 응용 프로그램 메뉴를 처리합니다.
//  WM_PAINT    - 주 창을 그립니다.
//  WM_DESTROY  - 종료 메시지를 게시하고 반환합니다.
//
//
RECT crt;
RECT window = { left, bottom, right, top };
POINT pt;
Vec2 vector;
Vec2 vecOld;
bool mouseClick = 0;
UINT commandFlag = 1;
bool ArtificialGravityFlag = 0;

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	HDC hdc, hMemDC;
	PAINTSTRUCT ps;
	HBITMAP hBitmap, OldBitmap;
	HBRUSH Brush, oBrush;

	switch (message)
	{
	case WM_CREATE:
		MoveWindow(hWnd, 100, 100, 1200, 700, TRUE);
		GetClientRect(hWnd, &crt);
		SetTimer(hWnd, 0, tick, NULL);
		return FALSE;
		break;
	case WM_GETMINMAXINFO:
		((MINMAXINFO*)lParam)->ptMaxTrackSize.x = right + 50;
		((MINMAXINFO*)lParam)->ptMaxTrackSize.y = bottom + 100;
		((MINMAXINFO*)lParam)->ptMinTrackSize.x = right + 50;
		((MINMAXINFO*)lParam)->ptMinTrackSize.y = bottom + 100;
		return FALSE;
		break;
	case WM_COMMAND:
	{
		int wmId = LOWORD(wParam);
		// 메뉴 선택을 구문 분석합니다.
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
	}
	break;
	case WM_KEYDOWN:
		switch (wParam) {
		case VK_ESCAPE:
			DestroyWindow(hWnd);
			break;
		case VK_SPACE:
			toggle = !toggle;
			break;
		case VK_UP:
			tick++;
			tick++;
			if (tick > 100)
				tick = 100;
			fps = 1000 / tick;
			KillTimer(hWnd, 0);
			SetTimer(hWnd, 0, tick, NULL);
			//fps += 10;
			//if (fps > 1000)
			//	fps = 1000;
			//tick = 1000 / fps;
			break;
		case VK_DOWN:
			tick--;
			tick--;
			if (tick < 1)
				tick = 1;
			fps = 1000 / tick;
			KillTimer(hWnd, 0);
			SetTimer(hWnd, 0, tick, NULL);
			break;
		case VK_INSERT:
			GetCursorPos(&pt);
			ScreenToClient(hWnd, &pt);
			if (objnum > maxobj - 1) break;
			else {
				objnum++;
				index++;
				objList[index] = new Object(rand() % 5 + 10, Vec2(pt.x + rand() % 20 - 10, pt.y), 0.8, 5, Vec2());
				break;
			}
		case VK_RIGHT:
			objList[8]->torqueImpulse += 100000;
			break;
		case VK_LEFT:
			objList[8]->torqueImpulse -= 100000;
			break;
		case VK_NUMPAD1:
			if (commandFlag == 1) commandFlag = 2;
			else if (commandFlag == 2) commandFlag = 1;
			break;
		}
		break; // 지우지 말것....
	case WM_LBUTTONDOWN:
		if (commandFlag == 1) {
			if (objnum > maxobj - 1) break;
			else {
				GetCursorPos(&pt);
				ScreenToClient(hWnd, &pt);
				if (pt.x > right || pt.x < left || pt.y > bottom || pt.y < top) {
					mouseClick = 0;
					break;
				}
				else if (mouseClick == 0) {
					vector = Vec2(pt.x, pt.y);
					mouseClick = 1;
					break;
				}
			}
		}
		else if (commandFlag == 2) {
			ArtificialGravityFlag = 1;
		}
		break;
	case WM_MOUSEMOVE:
		/*if (mouseClick == 1) {
			if (objnum > maxobj - 1) break;
			else {
				objnum++;
				index++;
				objList[index] = new Object(rand() % 5 + 5, Vec2(LOWORD(lParam) + rand() % 20 - 10, HIWORD(lParam)), 0.2, 2, Vec2());
				break;
			}
		}*/
		break;
	case WM_LBUTTONUP:
		if (commandFlag == 1) {
			if (objnum > maxobj - 1) {
				mouseClick = 0;
				break;
			}
			else if (mouseClick == 1) {
				GetCursorPos(&pt);
				ScreenToClient(hWnd, &pt);
				vecOld = vector;
				vector = (Vec2(pt.x, pt.y) - vector) * 2;
				objnum++;
				index++;
				//objList[index] = new Object(rand() % 5 + 10, Vec2(vecOld.X() + rand() % 20 - 10, vecOld.Y()), 0.2, 2, vector);
				objList[index] = new Object(Vec2(vecOld.X() - 20 + rand() % 10, vecOld.Y() - 20 + rand() % 10),
					Vec2(vecOld.X() + 20 - rand() % 10, vecOld.Y() + 20 - rand() % 10), 0.4, 8, vector);
				mouseClick = 0;
				break;
			}
			else
				break;
		}
		else if (commandFlag == 2) {
			ArtificialGravityFlag = 0;
		}
		break;
	case WM_TIMER:
		InvalidateRect(hWnd, NULL, false);
		//nTime++;
		break;
	case WM_PAINT:
	{
		UINT oldTime = st.wMilliseconds;
		if (toggle == 1) {
			hdc = BeginPaint(hWnd, &ps);
			hMemDC = CreateCompatibleDC(hdc);
			hBitmap = CreateCompatibleBitmap(hdc, crt.right, crt.bottom);
			OldBitmap = (HBITMAP)SelectObject(hMemDC, hBitmap);

			Rectangle(hMemDC, left - 300, top - 300, right + 300, bottom + 300);
			sprintf_s(szBuf, "%d시 %d분 %d초", st.wHour, st.wMinute, st.wSecond);
			TextOut(hMemDC, 0, 0, szBuf, strlen(szBuf));
			sprintf_s(szBuf, "Tick : %d ms", tick);
			TextOut(hMemDC, 0, 20, szBuf, strlen(szBuf));
			sprintf_s(szBuf, "FPS: <%d", fps);
			TextOut(hMemDC, 0, 40, szBuf, strlen(szBuf));
			sprintf_s(szBuf, "Object: %d", objnum);
			TextOut(hMemDC, 120, 0, szBuf, strlen(szBuf));
			sprintf_s(szBuf, "Manifold: %d", mannum);
			TextOut(hMemDC, 120, 20, szBuf, strlen(szBuf));
			if (commandFlag == 1) sprintf_s(szBuf, "<Object add Mode>  Toggle : Numpad1 / Command: LeftClick");
			else if (commandFlag == 2) sprintf_s(szBuf, "<Artificial Gravity Mode>  Toggle : Numpad1 / Command: LeftClick");
			TextOut(hMemDC, 120, 40, szBuf, strlen(szBuf));

			oldTime = st.wMilliseconds;

			Rectangle(hMemDC, left, top, right, bottom);

			if (oldmannum == 0) {
				mannum = objnum * (objnum - 1) / 2;
				manifoldList = new Manifold[mannum];
				makeManifold(objList, manifoldList);
				oldmannum = mannum;
			}
			if (oldobjnum != objnum) {
				mannum = objnum * (objnum - 1) / 2;
				oldobjnum = objnum;
				delete[] manifoldList;
				manifoldList = new Manifold[mannum];
				makeManifold(objList, manifoldList);
			}
			manifoldCollision(manifoldList); // manifold 충돌 연산
			//ArtificialGravityON(1500000, (left + 3 * right) / 4, (top + bottom) / 2);
			if (ArtificialGravityFlag == 1) {
				ArtificialGravityOnMouse(1500000, hWnd);
			}
;			VerticalGravityON(gravity);

			for (int i = 0; i < objnum; i++) {
				if (i == 0) {
					objList[i]->impulse = Vec2();
					objList[i]->velocity = Vec2(0, 0);
					objList[i]->angularVelocity = 1000 / 5 * 0.005;
				}
				else if (i == 5 || i == 6 || i == 7) {
					objList[i]->impulse = Vec2();
					objList[i]->velocity = Vec2(0, 0);
					objList[i]->angularVelocity = 1000 / 5 * 0.1;
				}
				else if (i == 1 || i == 3 || i == 4) {
					objList[i]->impulse = Vec2();
					objList[i]->velocity = Vec2(0, 0);
					objList[i]->angularVelocity = 1000 / 5 * 0;
				}
				applyImpulse(objList[i]);
				wallref(objList[i], left, top, right, bottom);
				makeShape(hMemDC, objList[i]);
			}

			TextOut(hMemDC, int(objList[0]->position.x) - 5, int(objList[0]->position.y) - 5, "1", strlen("1"));
			TextOut(hMemDC, int(objList[1]->position.x) - 5, int(objList[1]->position.y) - 5, "2", strlen("2"));
			TextOut(hMemDC, int(objList[2]->position.x) - 5, int(objList[2]->position.y) - 5, "3", strlen("3"));
			TextOut(hMemDC, int(objList[3]->position.x) - 5, int(objList[3]->position.y) - 5, "4", strlen("4"));
			TextOut(hMemDC, int(objList[4]->position.x) - 5, int(objList[4]->position.y) - 5, "5", strlen("5"));
			TextOut(hMemDC, int(objList[5]->position.x) - 5, int(objList[5]->position.y) - 5, "1", strlen("1"));
			TextOut(hMemDC, int(objList[6]->position.x) - 5, int(objList[6]->position.y) - 5, "2", strlen("2"));
			TextOut(hMemDC, int(objList[7]->position.x) - 5, int(objList[7]->position.y) - 5, "3", strlen("3"));
			TextOut(hMemDC, int(objList[8]->position.x) - 5, int(objList[8]->position.y) - 5, "4", strlen("4"));

			sprintf_s(szBuf, "(%d, %d)", pt.x - left, pt.y - top);
			TextOut(hMemDC, pt.x + 15, pt.y, szBuf, strlen(szBuf));
			MoveToEx(hMemDC, pt.x - 10, pt.y, NULL);
			LineTo(hMemDC, pt.x + 10, pt.y);
			MoveToEx(hMemDC, pt.x, pt.y - 10, NULL);
			LineTo(hMemDC, pt.x, pt.y + 10);

			Brush = CreateSolidBrush(RGB(255, 100, 100)); // 도형 색상 지정
			oBrush = (HBRUSH)SelectObject(hMemDC, Brush);

			for (UINT32 i = 0; i < mannum; i++) {
				if (manifoldList[i].A->Sleep == 0 || manifoldList[i].B->Sleep == 0) {
					if (manifoldList[i].colPoint != Vec2()) {
						Ellipse(hMemDC, manifoldList[i].colPoint.x - 3, manifoldList[i].colPoint.y - 3,
							manifoldList[i].colPoint.x + 3, manifoldList[i].colPoint.y + 3);
					}
				}
			}

			GetCursorPos(&pt);
			ScreenToClient(hWnd, &pt);
			if (pt.x > right + 50 || pt.x < 0 || pt.y > bottom + 50 || pt.y < 0)
				mouseClick = 0;
			if (mouseClick == 1) {
				Vec2 line = Vec2(pt.x - vector.X(), pt.y - vector.Y());
				MoveToEx(hMemDC, vector.X(), vector.Y(), NULL);
				LineTo(hMemDC, line.X() + vector.X(), line.Y() + vector.Y());
			}

			GetLocalTime(&st);
			sprintf_s(szBuf, "Delay: %d ms", st.wMilliseconds - oldTime);
			TextOut(hMemDC, 0, 60, szBuf, strlen(szBuf));

			// Double Buffering
			BitBlt(hdc, 0, 0, crt.right, crt.bottom, hMemDC, 0, 0, SRCCOPY);
			DeleteObject(SelectObject(hMemDC, OldBitmap));
			DeleteDC(hMemDC);

			SelectObject(hdc, oBrush);
			DeleteObject(Brush);

			EndPaint(hWnd, &ps);
		}
	}
	break;
	case WM_ERASEBKGND:
		return TRUE;
		break;
	case WM_DESTROY:
		KillTimer(hWnd, 0);
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// 정보 대화 상자의 메시지 처리기입니다.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}
