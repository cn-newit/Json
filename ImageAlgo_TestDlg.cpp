
// ImageAlgo_TestDlg.cpp : implementation file
//

#include "stdafx.h"
#include "ImageAlgo_Test.h"
#include "ImageAlgo_TestDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CImageAlgo_TestDlg dialog




CImageAlgo_TestDlg::CImageAlgo_TestDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CImageAlgo_TestDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_pNewBuffer = nullptr;
	m_pOriginBuffer = nullptr;
	m_bShow = false;
	m_pImageAlgo = nullptr;
	for (int i = 0; i < 9; ++i)
	{
		m_iTemplate[i] = 1;
	}
}

CImageAlgo_TestDlg::~CImageAlgo_TestDlg()
{
	if (m_pImageAlgo)
	{
		TW_ReleaseImageAlgo(m_pImageAlgo);
		m_pImageAlgo = nullptr;
	}

	if (m_pNewBuffer)
	{
		delete[] m_pNewBuffer;
		m_pNewBuffer = nullptr;
	}

	if (m_pOriginBuffer)
	{
		delete[] m_pOriginBuffer;
		m_pOriginBuffer = nullptr;
	}

	//關閉GDI+圖像庫;
	Gdiplus::GdiplusShutdown(m_GdiplusToken);
}

void CImageAlgo_TestDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_IMAGEALGO, m_comboCtrl);
}

BEGIN_MESSAGE_MAP(CImageAlgo_TestDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_LOAD, &CImageAlgo_TestDlg::OnBnClickedBtnLoad)
	ON_CBN_SELCHANGE(IDC_COMBO_IMAGEALGO, &CImageAlgo_TestDlg::OnCbnSelchangeComboImagealgo)
	ON_BN_CLICKED(IDC_BTN_SAVE, &CImageAlgo_TestDlg::OnBnClickedBtnSave)
END_MESSAGE_MAP()


// CImageAlgo_TestDlg message handlers

BOOL CImageAlgo_TestDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here

	InitCombo();
	Gdiplus::GdiplusStartup(&m_GdiplusToken, &m_GdiplusStartupInput, nullptr);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CImageAlgo_TestDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		if (m_bShow)
		{
			ShowImage(IDC_STATIC_PPICSHOW, m_pNewBuffer, m_iWidth, m_iHeight);
			ShowImage(IDC_STATIC_PICSHOW, m_pOriginBuffer, m_iWidth, m_iHeight);
		}

		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CImageAlgo_TestDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CImageAlgo_TestDlg::OnBnClickedBtnLoad()
{
	// TODO: Add your control notification handler code here
	CFileDialog file(TRUE, _T("*"), _T("*.bmp"), OFN_OVERWRITEPROMPT, _T("BMP文件(*.bmp)|*.bmp||"), NULL);

	if (file.DoModal() != IDOK)
	{
		MessageBox(_T("You have to load the image first!"), _T("ERROR"));
		return;
	}

	tstring stPath = file.GetPathName();
	if (!LoadImage(stPath))
		return;
	
	ShowImage(IDC_STATIC_PICSHOW, m_pOriginBuffer, m_iWidth, m_iHeight);
	m_pNewBuffer = new UCHAR[m_iWidth * m_iHeight * 3];
	memset(m_pNewBuffer, 0, m_iWidth * m_iHeight * 3);
	TW_CreateImageAlgo(&m_pImageAlgo, m_pOriginBuffer, m_iWidth, m_iHeight);
}

bool CImageAlgo_TestDlg::LoadImage(tstring &stPath)
{
	FILE *fp = nullptr;
	int error = _wfopen_s(&fp, stPath.c_str(), _T("rb"));

	if (error != 0)
	{
		MessageBox(_T("Open file fail!"), _T("ERROR"));
		return false;
	}

	BITMAPFILEHEADER fileHead = {0};
	BITMAPINFOHEADER infoHead = {0};
	fread(&fileHead, sizeof(fileHead), 1, fp);
	fread(&infoHead, sizeof(infoHead), 1, fp);

	if (infoHead.biBitCount == 24)
	{
		m_iWidth = infoHead.biWidth;
		m_iHeight = infoHead.biHeight;
		m_pOriginBuffer = new UCHAR[m_iWidth * m_iHeight * 3];
		memset(m_pOriginBuffer, 0, m_iWidth * m_iHeight * 3);
		fread(m_pOriginBuffer, 1, infoHead.biHeight*infoHead.biWidth*3, fp);
	}

	fclose(fp);
	return true;
}


void CImageAlgo_TestDlg::ShowImage(UINT uID, UCHAR *pBuffer, int &w, int &h)
{
	BITMAPINFOHEADER infoHeader = {0};
	infoHeader.biHeight = h;
	infoHeader.biWidth = w;
	infoHeader.biBitCount = 24;
	infoHeader.biClrImportant = 0x00000000;
	infoHeader.biClrUsed = 0x00000000;
	infoHeader.biCompression = BI_RGB;
	infoHeader.biPlanes = 1;
	infoHeader.biSize = sizeof(BITMAPINFOHEADER);
	infoHeader.biSizeImage = ((((UINT)infoHeader.biBitCount*infoHeader.biWidth+31)& ~31)/8)*infoHeader.biHeight;
	infoHeader.biXPelsPerMeter = 0;
	infoHeader.biYPelsPerMeter = 0;
	BITMAPINFO *pBmpInfo = nullptr;          // 记录图像信息      
	pBmpInfo = (BITMAPINFO*)new char[sizeof(BITMAPINFOHEADER)];
	// 为图像数据申请空间;  
	memcpy(pBmpInfo, &infoHeader, sizeof(BITMAPINFOHEADER));
	// 显示图像;
	CWnd *pwnd = GetDlgItem(uID);
	CRect rect;
	pwnd->GetClientRect(&rect);
	CDC *pDC = pwnd->GetDC();        // 获取picture控件所在的位置  ;
	pDC->SetStretchBltMode(COLORONCOLOR);
	StretchDIBits(pDC->GetSafeHdc(), 0, 0, rect.Width(), rect.Height(), 0, 0,
		w, h, pBuffer, pBmpInfo, DIB_RGB_COLORS, SRCCOPY);
	ReleaseDC(pDC);

	if (pBmpInfo)
	{
		delete[] pBmpInfo;
		pBmpInfo = nullptr;
	}
}

void CImageAlgo_TestDlg::InitCombo()
{
	m_comboCtrl.ResetContent();
	m_comboCtrl.InsertString(0, _T("灰度圖"));
	m_comboCtrl.InsertString(1, _T("灰度綫性變換"));
	m_comboCtrl.InsertString(2, _T("灰度閾值變換"));
	m_comboCtrl.InsertString(3, _T("腐蝕"));
	m_comboCtrl.InsertString(4, _T("膨胀"));
	m_comboCtrl.InsertString(5, _T("邊緣檢測梯度算子"));
	m_comboCtrl.InsertString(6, _T("邊緣檢測Roberts算子"));

//	m_comboCtrl.SetCurSel(-1);
}

void CImageAlgo_TestDlg::OnCbnSelchangeComboImagealgo()
{
	// TODO: Add your control notification handler code here
	
	int index = m_comboCtrl.GetCurSel();
	memset(m_pNewBuffer, 0, m_iWidth * m_iHeight * 3);
	if (m_pImageAlgo)
	{
		switch (index)
		{
		case 0:
			{
				m_pImageAlgo->RGB2GRAY(m_pNewBuffer);
				break;
			}
		case 1:
			{
				m_pImageAlgo->LinerTrans(m_pNewBuffer, 2, 50);
				break;
			}
			
		case 2:
			{
				m_pImageAlgo->ThresholdTrans(m_pNewBuffer, 100);
				break;
			}
			
		case 3:
			{
				m_pImageAlgo->Erosiontion(m_pNewBuffer, m_iTemplate, 3);
				break;
			}
		case 4:
			{
				m_pImageAlgo->Dilate(m_pNewBuffer, m_iTemplate, 3);
				break;
			}
		case 5:
			{
				m_pImageAlgo->G_RFunc(m_pNewBuffer);
				break;
			}
		case 6:
			{
				m_pImageAlgo->G_RFunc(m_pNewBuffer, 1, 1);
				break;
			}
		default:
			break;
		}

		if (m_pNewBuffer)
		{
			m_bShow = true;
		}

		ShowImage(IDC_STATIC_PPICSHOW, m_pNewBuffer, m_iWidth, m_iHeight);
	}
	
}

void CImageAlgo_TestDlg::DrawHistogram(UINT uID)
{
	CDC *pDc = GetDlgItem(uID)->GetDC();
	CRect rc;
	CDC memDc;
	CBitmap memBitmap;

	GetDlgItem(uID)->GetClientRect(&rc);
	//設備描述表初始化;
	memDc.CreateCompatibleDC(nullptr);
	//建立與屏幕顯示兼容的内存顯示設備;
	memBitmap.CreateCompatibleBitmap(pDc, rc.Width(), rc.Height());
	//選取空白位圖;
	memDc.SelectObject(memBitmap);
	memDc.FillSolidRect(0, 0, rc.Width(), rc.Height(), RGB(255, 255, 255));
	Gdiplus::Graphics graph(memDc.GetSafeHdc());
	//使用白色背景;
	graph.FillRectangles(&Gdiplus::SolidBrush(Gdiplus::Color::White), &Gdiplus::Rect(0, 0, rc.Width(), rc.Height()), 1);
	//繪製y軸;
	graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Black), 10, 10, 10, 280);
	graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Black), 10, 10, 5, 15);
	graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Black), 10, 10, 15, 15);

	//繪製x軸;
	graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Black), 10, 280, 290, 280);
	graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Black), 290, 280, 285, 285);
	graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Black), 290, 280, 285, 275);

	//繪製坐標原點;
	tstring stNum;
	Gdiplus::Font font(_T("宋体"), 10);
	stNum = _T("0");
	graph.DrawString(stNum.c_str(), -1, &font, Gdiplus::PointF(8, 290), &Gdiplus::SolidBrush(Gdiplus::Color::Black));

	for (int i = 0; i < 256; i+= 5)
	{
		if (i % 50 == 0)
		{
			graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Black), 10 + i, 280, 10 + i, 286);
		}
		else if (i % 10 == 0)
		{
			graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Black), 10 + i, 280, 10 + i, 283);
		}
	}

	//繪製x軸刻度;
	stNum = _T("50");
	graph.DrawString(stNum.c_str(), -1, &font, Gdiplus::PointF(53, 290), &Gdiplus::SolidBrush(Gdiplus::Color::Black));
	stNum = _T("100");
	graph.DrawString(stNum.c_str(), -1, &font, Gdiplus::PointF(100, 290), &Gdiplus::SolidBrush(Gdiplus::Color::Black));
	stNum = _T("150");
	graph.DrawString(stNum.c_str(), -1, &font, Gdiplus::PointF(150, 290), &Gdiplus::SolidBrush(Gdiplus::Color::Black));
	stNum = _T("200");
	graph.DrawString(stNum.c_str(), -1, &font, Gdiplus::PointF(200, 290), &Gdiplus::SolidBrush(Gdiplus::Color::Black));
	stNum = _T("255");
	graph.DrawString(stNum.c_str(), -1, &font, Gdiplus::PointF(255, 290), &Gdiplus::SolidBrush(Gdiplus::Color::Black));

	//繪製當前灰度區域;
	Gdiplus::Pen pen(Gdiplus::Color::Blue);
	pen.SetDashStyle(Gdiplus::DashStyleDash);
	graph.DrawLine(&pen, 10, 280, 10, 10);
	graph.DrawLine(&pen, 10 + 255, 280, 10 + 255, 10);
	long long lMax = 0;
	Gdiplus::REAL dHeight = 0.0;
	//查找最大值;
	for (int j = 0; j <= 255; ++j)
	{
		lMax = max(lMax, );
	}

	//y刻度;
	stNum = std::to_wstring(lMax);
	graph.DrawString(stNum.c_str(), -1, &font, Gdiplus::PointF(10, 25), &Gdiplus::SolidBrush(Gdiplus::Color::Black));

	//繪製柱狀圖;
	for (int n = 0; n <= 255; ++n)
	{
		dHeight = (Gdiplus::REAL)() / lMax * 250;
		graph.DrawLine(&Gdiplus::Pen(Gdiplus::Color::Gray), n + 10.0f, 280.0f, n + 10.0f, 280 - dHeight);
	}

	//複製内存畫布内容;
	pDc->BitBlt(0, 0, rc.Width(), rc.Height(), &memDc, 0, 0, SRCCOPY);
	GetDlgItem(uID)->ReleaseDC(pDc);
}

void CImageAlgo_TestDlg::OnBnClickedBtnSave()
{
	// TODO: Add your control notification handler code here
	SaveImg(m_pNewBuffer);
}

void CImageAlgo_TestDlg::SaveImg(UCHAR *pBuffer)
{
	BITMAPINFOHEADER infoHeader = { 0 };
	infoHeader.biSize = sizeof(BITMAPINFOHEADER);
	infoHeader.biHeight = m_iHeight;
	infoHeader.biWidth = m_iWidth;
	infoHeader.biPlanes = 1;
	infoHeader.biBitCount = 24;
	infoHeader.biCompression = BI_RGB;
	infoHeader.biClrImportant = 0x00000000;
	infoHeader.biClrUsed = 0x00000000;
	infoHeader.biSizeImage = ((((UINT)infoHeader.biBitCount*infoHeader.biWidth + 31)& ~31) / 8)*infoHeader.biHeight;
	infoHeader.biXPelsPerMeter = 0;
	infoHeader.biYPelsPerMeter = 0;

	BITMAPFILEHEADER bmpFileHeader = { 0 };
	bmpFileHeader.bfType = 0x4D42;
	bmpFileHeader.bfSize = sizeof(BITMAPFILEHEADER) + infoHeader.biSizeImage+sizeof(BITMAPINFOHEADER);//sizeof(BITMAPINFOHEADER) + m_width*m_height * 3;
	bmpFileHeader.bfReserved1 = 0;
	bmpFileHeader.bfReserved2 = 0;
	bmpFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	FILE *file = _tfopen(tstring(_T("D:\\bin.bmp")).c_str(), _T("wb"));
	if (file == nullptr)
		return;
	fwrite(&bmpFileHeader, 1, sizeof(BITMAPFILEHEADER), file);
	fwrite(&infoHeader, 1, sizeof(BITMAPINFOHEADER), file);
	fwrite(pBuffer, 1, m_iWidth*m_iHeight * 3, file);
	fclose(file);
}
