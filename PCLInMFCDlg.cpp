
// PCLInMFCDlg.cpp : implementation file
//

#include "stdafx.h"
#include "PCLInMFC.h"
#include "PCLInMFCDlg.h"
#include "afxdialogex.h"


#ifdef _DEBUG
//#define new DEBUG_NEW  //與pcl中new衝突;
#endif


// CPCLInMFCDlg dialog

//pcl::visualization::CloudViewer viewer("pcd viewer");
pcl::visualization::PCLVisualizer g_viewerS("", false);
pcl::visualization::PCLVisualizer g_viewerT("", false);
pcl::visualization::PCLVisualizer g_viewerR("", false);


CPCLInMFCDlg::CPCLInMFCDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CPCLInMFCDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_cloudTag = PointCloudT::Ptr(new PointCloudT());
	m_cloudSrc = PointCloudT::Ptr(new PointCloudT());
	m_cloudResult = PointCloudT::Ptr(new PointCloudT());
}

void CPCLInMFCDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT_SHOWMSG, m_editMsg);
}

BEGIN_MESSAGE_MAP(CPCLInMFCDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_LOAD, &CPCLInMFCDlg::OnBnClickedBtnLoad)
	ON_BN_CLICKED(IDC_BTN_ICP, &CPCLInMFCDlg::OnBnClickedBtnIcp)
	ON_BN_CLICKED(R, &CPCLInMFCDlg::OnBnClickedR)
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDC_BTN_SAVEICP, &CPCLInMFCDlg::OnBnClickedBtnSaveicp)
	ON_BN_CLICKED(IDC_BTN_EXT, &CPCLInMFCDlg::OnBnClickedBtnExt)
END_MESSAGE_MAP()


// CPCLInMFCDlg message handlers

BOOL CPCLInMFCDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	
	initPCLWindow(g_viewerS, &m_pRenderWndItS, IDC_STATIC_SRC);
	initPCLWindow(g_viewerT, &m_pRenderWndItT, IDC_STATIC_TAG);
	initPCLWindow(g_viewerR, &m_pRenderWndItR, IDC_STATIC_RESULT);
	RECT rectExt;
	GetWindowRect(&rectExt);
	MoveWindow(rectExt.left, rectExt.top, rectExt.right - rectExt.left - 290, rectExt.bottom - rectExt.top);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CPCLInMFCDlg::OnPaint()
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
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CPCLInMFCDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

bool CPCLInMFCDlg::loadFile(std::vector<tstring> &vcPath)
{
	CFileDialog dlg(TRUE, NULL, NULL, OFN_ALLOWMULTISELECT, NULL);
	TCHAR *fileBuffer = new TCHAR[1024];
	memset(fileBuffer, 0, 1024);
	dlg.m_ofn.nMaxFile = 1024;
	dlg.m_ofn.lpstrFile = fileBuffer;
	dlg.m_ofn.lpstrFile[0] = NULL;

	if (IDOK == dlg.DoModal())
	{
		tstring strFilePath = dlg.m_ofn.lpstrFile;

		POSITION Pos = dlg.GetStartPosition();

		while (Pos != NULL)
		{
			tstring strFileName = dlg.GetNextPathName(Pos);

			if (strFileName.find(_T(".pcd")) != std::string::npos)
			{
				vcPath.push_back(strFileName);
			}	
		}

		if (vcPath.empty() || (vcPath.size() < 2))
		{
			showResultMsg(tstring(_T("Please load two file at least!")));
			return false;
		}

		delete[] fileBuffer;
		return true;
	}
	else
	{
		delete[] fileBuffer;
		return false;
	}
}

void CPCLInMFCDlg::OnBnClickedBtnLoad()
{
	// TODO:  在此添加控件通知处理程序代码

	std::vector<tstring> vcPath;
	if (!loadFile(vcPath))
	{
		return;
	}

#if 1
	if (pcl::io::loadPCDFile<PointT>(std::string(vcPath.at(0).begin(), vcPath.at(0).end()), *m_cloudTag) == -1) 
	{
		TCHAR tcTemp[56] = { 0 };
		_stprintf(tcTemp, _T("Load \"%s\" failed."), vcPath.at(0));
		showResultMsg(tstring(tcTemp));
		return;
	}

	TCHAR tcCountT[64] = { 0 };
	_stprintf(tcCountT, _T("Tag point cloud size is %d."), m_cloudTag->size());
	showResultMsg(tstring(tcCountT));

	if (pcl::io::loadPCDFile<PointT>(std::string(vcPath.at(1).begin(), vcPath.at(1).end()), *m_cloudSrc) == -1)
	{
		TCHAR tcTemp[56] = { 0 };
		_stprintf(tcTemp, _T("Load \"%s\" failed."), vcPath.at(1));
		showResultMsg(tstring(tcTemp));
		return;
	}

	TCHAR tcCountS[64] = { 0 };
	_stprintf(tcCountS, _T("Src point cloud size is %d."), m_cloudSrc->size());
	showResultMsg(tstring(tcCountS));

	showPointCloud(g_viewerS, m_pRenderWndItS, m_cloudSrc, std::string("src_cloud"));
	showPointCloud(g_viewerT, m_pRenderWndItT, m_cloudTag, std::string("tag_cloud"));

#else
	int size = vcPath.size();

	for (int i = 0; i < size; ++i)
	{
		if (i == 0)
		{
			if (pcl::io::loadPCDFile<PointT>(std::string(vcPath.at(0).begin(), vcPath.at(0).end()), *m_cloudTag) == -1)
			{
				TCHAR tcTemp[56] = { 0 };
				_stprintf(tcTemp, _T("Load \"%s\" failed."), vcPath.at(0));
				showResultMsg(tstring(tcTemp));
				return;
			}
		}
		else
		{
			if (pcl::io::loadPCDFile<PointT>(std::string(vcPath.at(i).begin(), vcPath.at(i).end()), *m_cloudSrc) == -1)
			{
				TCHAR tcTemp[56] = { 0 };
				_stprintf(tcTemp, _T("Load \"%s\" failed."), vcPath.at(0));
				showResultMsg(tstring(tcTemp));
				return;
			}

			if (!m_cloudResult->empty())
			{
				pcl::copyPointCloud(*m_cloudResult, *m_cloudTag);
			}
			
		}


		if (!m_cloudTag->empty() && !m_cloudSrc->empty())
		{
			showPointCloud(g_viewerS, m_pRenderWndItS, m_cloudSrc, std::string("src_cloud"));
			showPointCloud(g_viewerT, m_pRenderWndItT, m_cloudTag, std::string("tag_cloud"));
			Registration(true);
			showPointCloud(g_viewerR, m_pRenderWndItR, m_cloudResult, std::string("res"));
		}
	}

	showResultMsg(tstring(_T("Registration done!")));

#endif
}


void CPCLInMFCDlg::OnBnClickedBtnIcp()
{
	// TODO:  在此添加控件通知处理程序代码

	int iTemp = GetDlgItemInt(IDC_EDIT_ITERCOUNT);
	if (iTemp != 0)
	{
		m_iterationsCount = iTemp;
	}
	
	CString stDistanceThreshold = _T("");
	GetDlgItemText(IDC_EDIT_DIST, stDistanceThreshold);

	if (wcscmp(stDistanceThreshold, _T("")) != 0)
	{
		m_distanceThreshold = _ttof(stDistanceThreshold);
	}
	//
	CString str1 = _T("");
	GetDlgItemText(IDC_EDIT_LOWER, str1);

	if (wcscmp(str1, _T("")) != 0)
	{
		m_dFilterLimitMin = _ttof(str1);
	}

	CString str2 = _T("");
	GetDlgItemText(IDC_EDIT_UPPER, str2);

	if (wcscmp(str2, _T("")) != 0)
	{
		m_dFilterLimitMax = _ttof(str2);
	}

	CString str3 = _T("");
	GetDlgItemText(IDC_EDIT_AXES, str3);

	if (wcscmp(str3, _T("")) != 0)
	{
		tstring strTemp = str3;
		m_strFieldName = std::string(strTemp.begin(), strTemp.end());
	}

	CString str4 = _T("");
	GetDlgItemText(IDC_EDIT_DISTS, str4);

	if (wcscmp(str4, _T("")) != 0)
	{
		m_dCorresDistanceThshold = _ttof(str4);
	}
	
	int iTemp1 = GetDlgItemInt(IDC_EDIT_POINTCOUNT);

	if (iTemp1 != 0)
	{
		m_iCalcNumOfNuberPoint = iTemp1;
	}

	int iTemp2 = GetDlgItemInt(IDC_EDIT_ITERCOUNTS);

	if (iTemp2 != 0)
	{
		m_iSACIAIterationCount = iTemp2;
	}

	Registration(false);
	showPointCloud(g_viewerR, m_pRenderWndItR, m_cloudResult, std::string("res"));
	
}

void CPCLInMFCDlg::downSample(PointCloudT::Ptr cloudIn, PointCloudT::Ptr cloudOut)
{
	pcl::VoxelGrid<PointT> downSampled;
	downSampled.setInputCloud(cloudIn);
	downSampled.setLeafSize(0.03f, 0.03f, 0.03f);  //设置滤波时创建的体素体积为1cm的立方体（1为米，0.01就是1cm）;
	downSampled.filter(*cloudOut);
}

void CPCLInMFCDlg::passThrough(PointCloudT::Ptr cloudIn, PointCloudT::Ptr cloudOut, std::string strFieldName, double filterLimitMin, double filterLimitMax)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloudIn);
	pass.setFilterFieldName(strFieldName);
	pass.setFilterLimits(filterLimitMin, filterLimitMax);
//	pass.setFilterFieldName("y");
//	pass.setFilterLimits(0, filterLimit);
//	pass.setFilterFieldName("z");
//	pass.setFilterLimits(0, filterLimit);//保留或过滤z轴方向0到filterLimit;
	pass.filter(*cloudOut);
//	showPointCloud(g_viewerR, m_pRenderWndItR, cloudOut, std::string("filter"));
}

void CPCLInMFCDlg::removeOutlierPoint(PointCloudT::Ptr cloudIn, PointCloudT::Ptr cloudOut)
{
	////统计分析;
	//离群点是按照K个近邻点的标准方差*Threshold 来定义的，假如K = 50, Threshold = 1. 以下为确定某一点是离群点的算法;
	//首先求出这个点附近的50个点之间距离的标准方差dev，然后计算这个点到这些点的距离d，如果d > dev*threshold 那么这个点就是离群点;
	
	pcl::StatisticalOutlierRemoval<PointT> sor;   
	sor.setInputCloud(cloudIn);
	sor.setMeanK(50);                               //设置在进行统计时考虑的临近点个数;
	sor.setStddevMulThresh(/*1.0*/1.8);                      //设置判断是否为离群点的阀值，用来倍乘标准差;
	sor.filter(*cloudOut);

	/*pcl::RadiusOutlierRemoval<PointT> ror;  
	ror.setInputCloud(cloudIn);
	ror.setRadiusSearch(0.03);               // 设置搜索半径;
	ror.setMinNeighborsInRadius(3);      // 设置一个内点最少的邻居数目;
	ror.filter(*cloudOut);*/
	
}

void CPCLInMFCDlg::getNormals(PointCloudT::Ptr cloudIn, PointCloudWithNormals &normalsOut)
{	
	pcl::NormalEstimation<PointT, PointNormalT> norm;
	norm.setInputCloud(cloudIn);
	pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>());
	norm.setSearchMethod(kdTree);
//	norm.setKSearch(5);//5,10,15,20,35
	norm.setRadiusSearch(0.02);
	norm.compute(normalsOut);//計算法向量;

}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr CPCLInMFCDlg::getFeatures(PointCloudT::Ptr cloudIn, PointCloudWithNormals::Ptr normalsIn)
{
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
	
	pcl::FPFHEstimation<PointT, PointNormalT, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloudIn);
	fpfh.setInputNormals(normalsIn);
	pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>());
	kdTree->setInputCloud(cloudIn);
	fpfh.setSearchMethod(kdTree);
//	fpfh.setKSearch(30);
	fpfh.setRadiusSearch(0.03);
	fpfh.compute(*features);
	return features;
}

void CPCLInMFCDlg::sacAlignment(PointCloudT::Ptr tag, PointCloudT::Ptr src, pcl::PointCloud<pcl::FPFHSignature33>::Ptr featuresTag,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr featuresSrc, Eigen::Matrix4f &matrix)
{
	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(src);
	sac_ia.setInputTarget(tag);
	sac_ia.setSourceFeatures(featuresSrc);
	sac_ia.setTargetFeatures(featuresTag);
	sac_ia.setMaxCorrespondenceDistance(m_dCorresDistanceThshold);//
	sac_ia.setCorrespondenceRandomness(m_iCalcNumOfNuberPoint);//计算协方差时选择多少各临近点计算;
	sac_ia.setMaximumIterations(m_iSACIAIterationCount);
	PointCloudT::Ptr finalcloud(new PointCloudT);
	sac_ia.align(*finalcloud);

	if (sac_ia.hasConverged())
	{
		TCHAR *tcRes = new TCHAR[64];
		memset(tcRes, 0, 64 * sizeof(TCHAR));
		_stprintf(tcRes, _T("SAC has converged, score: %0.10f"), sac_ia.getFitnessScore());
		showResultMsg(tstring(tcRes));
		delete[] tcRes;
		tcRes = nullptr;
		matrix *= sac_ia.getFinalTransformation();
		print4x4Matrix(matrix);
		*finalcloud += *tag;
		showPointCloud(g_viewerR, m_pRenderWndItR, finalcloud, std::string("res"));
	}
	else
	{
		showResultMsg(tstring(_T("SAC-IA failed!")));
	}
}

void CPCLInMFCDlg::getRegionPoint(PointCloudT::Ptr cloudIn, PointCloudWithNormals::Ptr &normalsIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudOut)
{
	//基于区域生长算法;
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);//小于这个参数的平面被忽略;
	reg.setMaxClusterSize(100000);
	pcl::search::Search<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>());
	reg.setSearchMethod(kdTree);
	reg.setNumberOfNeighbours(50);
	reg.setInputCloud(cloudIn);
	//reg.setIndices (indices);
	reg.setInputNormals(normalsIn);
	//以下两个参数较为重要，对分割的结果影响最大;
	reg.setSmoothnessThreshold(4.0 / 180.0 * M_PI);//每个近邻点与当前种子点的法线角度差阈值（0.03490...,0.0523...）;两个法线在多大的夹角内还可以当做是共面的。
	reg.setCurvatureThreshold(10.0);//曲率阈值,越小弯曲程度越小，反之;

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	cloudOut = reg.getColoredCloud();

	int a = 0;

}

void CPCLInMFCDlg::getRegionPoint(PointCloudT::Ptr cloudIn, PointCloudT::Ptr &cloudOut)
{
	/*pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	// Optional
	seg.setOptimizeCoefficients(true);//设置对估计的模型系数需要进行优化;
	// Mandatory
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);//設置分割模型;
	seg.setNormalDistanceWeight(0.1);//设置表面法线权重系数;
	seg.setMethodType(pcl::SAC_RANSAC);//设置采用RANSAC作为算法的参数估计方法;
	seg.setMaxIterations(500); //设置迭代的最大次数;
	seg.setDistanceThreshold(0.2);//设置内点到模型的距离允许最大值;

	seg.setInputCloud(cloudIn);
	seg.setInputNormals(normalsIn);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.empty())
	{
	return;
	}

	// 提取地面;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloudIn);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloudOut);*/
}

bool CPCLInMFCDlg::Registration(bool bDownSample)
{
	//對點雲進行預處理和粗配准,以提供一個較好的初始位置讓ICP算法精配准;

	PointCloudT::Ptr cloudSrc(new PointCloudT);
	PointCloudT::Ptr cloudTag(new PointCloudT);
	Eigen::Matrix4f sacTrans = Eigen::Matrix4f::Identity();

	//下采樣濾波;
	if (bDownSample)
	{
		downSample(m_cloudSrc, cloudSrc);
		downSample(m_cloudTag, cloudTag);

		TCHAR tcSrc[64] = { 0 };
		_stprintf(tcSrc, _T("After downsample src size is %d. \r\n\t\ttag size is %d."), cloudSrc->size(), cloudTag->size());
		showResultMsg(tstring(tcSrc));
	}
	else
	{
		cloudSrc = m_cloudSrc;
		cloudTag = m_cloudTag;
	}

	//沿x軸旋轉點雲逆時針58度;
	/*Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	// Here we defined a 45° (PI/4) rotation around the X axis and a translation on the X axis.
	float theta = -M_PI / 3; // The angle of rotation in radians（≈60°）;
	transform_1(1, 1) = cos(theta);
	transform_1(1, 2) = -sin(theta);
	transform_1(2, 1) = sin(theta);
	transform_1(2, 2) = cos(theta);

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	// Here we defined a 45° (PI/4) rotation around the X axis and a translation on the X axis.
	float thetaZ = -M_PI / 12; // The angle of rotation in radians;
	transform_1(0, 0) = cos(thetaZ);
	transform_1(0, 1) = -sin(thetaZ);
	transform_1(1, 0) = sin(thetaZ);
	transform_1(1, 1) = cos(thetaZ);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrcT(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTagT(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloudSrc, *cloudSrcT, transform_1);
	pcl::transformPointCloud(*cloudTag, *cloudTagT, transform_1);
	pcl::transformPointCloud(*cloudSrcT, *cloudSrcT, transform);
	pcl::transformPointCloud(*cloudTagT, *cloudTagT, transform);
	showPointCloud(g_viewerS, m_pRenderWndItS, cloudSrcT, std::string("src"));
	showPointCloud(g_viewerT, m_pRenderWndItT, cloudTagT, std::string("tag"));*/

	
	//去除離群點;
	PointCloudT::Ptr cloudSrcO(new PointCloudT);
	PointCloudT::Ptr cloudTagO(new PointCloudT);
	removeOutlierPoint(cloudSrc, cloudSrcO);
	removeOutlierPoint(cloudTag, cloudTagO);
//	showPointCloud(g_viewerS, m_pRenderWndItS, cloudSrcO, std::string("srco"));
//	showPointCloud(g_viewerT, m_pRenderWndItT, cloudTagO, std::string("tago"));
	showResultMsg(tstring(_T("Remove outlier point cloud done.")));

	//直通滤波;
	PointCloudT::Ptr cloudSrcPS(new PointCloudT);
	PointCloudT::Ptr cloudTagPS(new PointCloudT);
	passThrough(cloudSrcO, cloudSrcPS, m_strFieldName, m_dFilterLimitMin, m_dFilterLimitMax);
	passThrough(cloudTagO, cloudTagPS, m_strFieldName, m_dFilterLimitMin, m_dFilterLimitMax);
	showPointCloud(g_viewerS, m_pRenderWndItS, cloudSrcPS, std::string("srco"));
	showPointCloud(g_viewerT, m_pRenderWndItT, cloudTagPS, std::string("tago"));
	showResultMsg(tstring(_T("Pass through point cloud done.")));

	//平滑点云;
	PointCloudT::Ptr cloudSrcSM(new PointCloudT);
	PointCloudT::Ptr cloudTagSM(new PointCloudT);
	smoothPointcloud(cloudSrcPS, cloudSrcSM);
	smoothPointcloud(cloudTagPS, cloudTagSM);
//	showPointCloud(g_viewerS, m_pRenderWndItS, cloudSrcSM, std::string("srco"));
//	showPointCloud(g_viewerT, m_pRenderWndItT, cloudTagSM, std::string("tago"));
	showResultMsg(tstring(_T("Smooth point cloud done.")));

	//計算表面法向量;
	PointCloudWithNormals::Ptr normalsSrc(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr normalsTag(new PointCloudWithNormals);
	getNormals(cloudSrcSM, *normalsSrc);
	getNormals(cloudTagSM, *normalsTag);
//	showPointCloudNormals(g_viewerS, m_pRenderWndItS, cloudSrcPS, normalsSrc, std::string("norS"));
//	showPointCloudNormals(g_viewerT, m_pRenderWndItT, cloudTagPS, normalsTag, std::string("norT"));
	showResultMsg(tstring(_T("Compute normal estimation done.")));

	//点云分割;
	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSrcK(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTagK(new pcl::PointCloud<pcl::PointXYZRGB>());
	getRegionPoint(cloudSrcSM, normalsSrc, cloudSrcK);
	getRegionPoint(cloudTagSM, normalsTag, cloudTagK);
	showPointCloudRGB(g_viewerS, m_pRenderWndItS, cloudSrcK);
	showPointCloudRGB(g_viewerT, m_pRenderWndItT, cloudTagK);*/

	//FPFH特徵估計;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhSrc = getFeatures(cloudSrcSM, normalsSrc);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhTag = getFeatures(cloudTagSM, normalsTag);
	showResultMsg(tstring(_T("Compute fpfh estimation done.")));

	/*pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
	crude_cor_est.setInputSource(fpfhSrc);
	crude_cor_est.setInputTarget(fpfhTag);
	//  crude_cor_est.determineCorrespondences(cru_correspondences);
	crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
	showCorrespondences(g_viewerR, m_pRenderWndItR, cloudTagPS, cloudSrcPS, *cru_correspondences);*/

	//SAC-IA粗配准(采樣一致性初始配准算法);
	/*sacAlignment(cloudTagSM, cloudSrcSM, fpfhTag, fpfhSrc, sacTrans);

	PointCloudT::Ptr cloudSrcT(new PointCloudT);
	pcl::transformPointCloud(*cloudSrcSM, *cloudSrcT, sacTrans);
	m_cloudResult = cloudTagSM;
	*m_cloudResult += *cloudSrcT;*/
	//ICP最近點迭代算法進行精配准;
	m_time.tic();
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	//參數設定;
	icp.setInputSource(cloudSrcSM);
	icp.setInputTarget(cloudTagSM);
	// Set the maximum number of iterations (criterion 1)//最大迭代次数;
	icp.setMaximumIterations(m_iterationsCount);
	// Set the transformation epsilon (criterion 2)设置两次变换矩阵之间的差值(一般设置为1e-10);
	icp.setTransformationEpsilon(1e-10);
	// 设置对应点对之间的最大距离,当点对距离大于此阈值时忽略该点对(影响因素较大);
	icp.setMaxCorrespondenceDistance(m_distanceThreshold);//shoe1-shoe5 == 0.002
	// Set the euclidean distance difference epsilon (criterion 3)设置收敛条件是均方误差和小于阈值，停止迭代;
	icp.setEuclideanFitnessEpsilon(/*0.00005*/1e-5);

	PointCloudT::Ptr cloudSrcNew(new PointCloudT);
	//執行配准;
	icp.align(*cloudSrcNew/*, sacTrans*/);

	if (icp.hasConverged())
	{
		TCHAR tcMsg[128] = { 0 };
		_stprintf(tcMsg, _T("Apply %d icp iteration spent %0.f ms. \r\nICP has converged, score is %0.20f!"), m_iterationsCount, m_time.toc(), icp.getFitnessScore());
		showResultMsg(tstring(tcMsg));
		sacTrans *= icp.getFinalTransformation();
//		transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
		print4x4Matrix(sacTrans);
		PointCloudT::Ptr cloudSrcTrs(new PointCloudT);
		pcl::transformPointCloud(*cloudSrc, *cloudSrcTrs, sacTrans);

		m_cloudResult = cloudTag;//m_cloudTag;
		*m_cloudResult += *cloudSrcTrs;
		return true;
	}
	else
	{
		showResultMsg(tstring(_T("ICP registration failed!")));
		return false;
	}
//	return true;
}

void CPCLInMFCDlg::print4x4Matrix(const Eigen::Matrix4f &matrix)
{
	TCHAR *tcMsg =  new TCHAR[1024];
	memset(tcMsg, 0, 1024);
	_stprintf(tcMsg, _T("Rotation matrix : \r\n  | %6.3f %6.3f %6.3f | \r\n  | %6.3f %6.3f %6.3f | \r\n  | %6.3f %6.3f %6.3f | \r\nTranslation vector : \r\n  | %6.3f %6.3f %6.3f |"),
		matrix(0, 0), matrix(0, 1), matrix(0, 2), 
		matrix(1, 0), matrix(1, 1), matrix(1, 2), 
		matrix(2, 0), matrix(2, 1), matrix(2, 2), 
		matrix(0, 3), matrix(1, 3), matrix(2, 3));
	showResultMsg(tstring(tcMsg));
	delete[] tcMsg;
	tcMsg = nullptr;
}

void CPCLInMFCDlg::showResultMsg(tstring &stMsg)
{
	m_editMsg.SetSel(-1);//
	tstring stTemp = stMsg + _T("\r\n");
	m_editMsg.ReplaceSel(stTemp.c_str());
}

void CPCLInMFCDlg::initPCLWindow(pcl::visualization::PCLVisualizer &viewer,vtkRenderWindowInteractor** ppRenderWndIt, UINT uID)
{
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(1.0);
	viewer.setShowFPS(false);
	viewer.resetCamera();
	vtkRenderWindow *pRenderWnd = viewer.getRenderWindow();
	(*ppRenderWndIt) = vtkRenderWindowInteractor::New();
	CRect rect;
	CStatic *pStatic = (CStatic*)GetDlgItem(uID);
	pStatic->GetWindowRect(&rect);
	pRenderWnd->SetParentId(pStatic->m_hWnd);
	pRenderWnd->SetSize(rect.Width(), rect.Height());
	pRenderWnd->SetPosition(0, 0);
	(*ppRenderWndIt)->SetRenderWindow(pRenderWnd);
	pRenderWnd->Render();
}

void CPCLInMFCDlg::showPointCloud(pcl::visualization::PCLVisualizer &viewer, vtkRenderWindowInteractor* pRenderWndIt, const PointCloudT::Ptr cloud, std::string &stID)
{
	viewer.removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<PointT> srcColor(cloud, 0, 255, 0);
	viewer.addPointCloud<PointT>(cloud, srcColor, stID);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, stID);
	pRenderWndIt->Render();
	/*while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/
}

void CPCLInMFCDlg::showPointCloudRGB(pcl::visualization::PCLVisualizer &viewer, vtkRenderWindowInteractor* pRenderWndIt,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	viewer.removeAllPointClouds();
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	pRenderWndIt->Render();
}

void CPCLInMFCDlg::showPolygonMesh(pcl::visualization::PCLVisualizer &viewer, vtkRenderWindowInteractor* pRenderWndIt, pcl::PolygonMesh &mesh, std::string &stID)
{
	viewer.removeAllPointClouds();
	viewer.removePolygonMesh(stID);
	viewer.addPolygonMesh(mesh, stID);

	pRenderWndIt->Render();
	/*while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/
}

void CPCLInMFCDlg::showPointCloudNormals(pcl::visualization::PCLVisualizer &viewer, vtkRenderWindowInteractor* pRenderWndIt, const PointCloudT::Ptr cloud, 
	PointCloudWithNormals::Ptr normalsSrc, std::string &stID)
{
	viewer.removeAllPointClouds();
	//cloud为原始点云模型，normal为法向信息，5表示需要显示法向的点云间隔，即每5个点显示一次法向，0.0５表示法向长度;
	viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud, normalsSrc, 5, 0.05, stID);
	pRenderWndIt->Render();
}


void CPCLInMFCDlg::showCorrespondences(pcl::visualization::PCLVisualizer &viewer, vtkRenderWindowInteractor* pRenderWndIt, const PointCloudT::Ptr tag, const PointCloudT::Ptr src,
	const pcl::Correspondences &correspondences)
{
	viewer.removeAllPointClouds();
	viewer.removeCorrespondences("correspond");
	//设置对应点连线的粗细,PCL_VISUALIZER_LINE_WIDTH,表示线操作,线段的宽度为2(线段的宽度最好不要超过自定义的点的大小),"correspond"表示对对应的标签做处理;
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "correspond"); 
	//设置对应点连线的颜色，范围从0-1之间;
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "correspond"); 
	viewer.addPointCloud<pcl::PointXYZ>(tag, "correspond");
	viewer.addPointCloud<pcl::PointXYZ>(src, "correspond");
	viewer.addCorrespondences<pcl::PointXYZ>(src, tag, correspondences, "correspond");
	pRenderWndIt->Render();
}

void CPCLInMFCDlg::reconstructPoint(const PointCloudT::Ptr cloud)
{
	m_time.tic();
	/*PointCloudT::Ptr cloudNew(new PointCloudT);
	removeOutlierPoint(cloud, cloudNew);
	showPointCloud(g_viewerR, m_pRenderWndItR, cloudNew, std::string("res"));*/

	//贪婪投影三角化算法;
	// Normal estimation*
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	getNormals(cloud, *normals);

	/*pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloudNew);
	n.setInputCloud(cloudNew);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); */


	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	//将点云和法线放到一起;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	/*不同的pcd文件，主要需要重设setSearchRadius与setMu;*/
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.25f);
	gp3.setMu(/*2.5*//*40.0*/60.0f);//设置最近邻距离的乘子，以得到每个点的最终搜索半径【默认值 0】,I3点云数据需设定较大数值;
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(/*false*/true);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(m_triangles);

/*
//泊松重建;
	// 计算法向量;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); 
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //计算法线，结果存储在normals中;

	//将点云和法线放到一起;
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	//创建搜索树;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::Poisson<pcl::PointNormal> pn;
	pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化;
	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久;
	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度;
	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度;
	pn.setManifold(false);
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）;
	pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑;
	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率;
	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度;
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);

	pn.performReconstruction(m_triangles);*/

	TCHAR tcTime[128] = { 0 };
	_stprintf(tcTime, _T("Reconstruct spent %0.f ms."), m_time.toc());
	showResultMsg(tstring(tcTime));
}

//重采样平滑点云;
void CPCLInMFCDlg::smoothPointcloud(PointCloudT::Ptr &cloudIn, PointCloudT::Ptr &cloudOut)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::MovingLeastSquares<PointT, PointT> mls; 
	mls.setSearchMethod(tree);  
	mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线;
	mls.setInputCloud(cloudIn);
	mls.setPolynomialOrder(4);    // 拟合2阶多项式拟合,3或4为佳;
	mls.setPolynomialFit(true);  // 设置为false可以 加速 smooth;
	mls.setSearchRadius(0.05); // 单位m.设置用于拟合的K近邻半径,愈大平滑效果越明显，但形状会发生改变;
	mls.process(*cloudOut);
//	mls.setUpsamplingMethod();
}


void CPCLInMFCDlg::saveAlignResult2PCD()
{
	pcl::io::savePCDFileBinary("alignRes.pcd", *m_cloudResult);
}


void CPCLInMFCDlg::OnBnClickedR()
{
	// TODO:  在此添加控件通知处理程序代码
	reconstructPoint(m_cloudResult);
	showPolygonMesh(g_viewerR, m_pRenderWndItR, m_triangles, std::string("triangles"));
}



void CPCLInMFCDlg::OnBnClickedBtnSaveicp()
{
	// TODO:  在此添加控件通知处理程序代码
	saveAlignResult2PCD();
}


void CPCLInMFCDlg::OnBnClickedBtnExt()
{
	// TODO:  在此添加控件通知处理程序代码
	CString strTmp = _T("");
	GetDlgItem(IDC_BTN_EXT)->GetWindowText(strTmp);
	if (strTmp == _T(">>"))
	{
		GetDlgItem(IDC_BTN_EXT)->SetWindowText(_T("<<"));
		RECT rect;
		GetWindowRect(&rect);
		MoveWindow(rect.left, rect.top, rect.right - rect.left + 290, rect.bottom - rect.top);
//		GetDlgItem(IDC_EDIT_PARAMS)->SetFocus();
	}
	else
	{
		GetDlgItem(IDC_BTN_EXT)->SetWindowText(_T(">>"));
		RECT rect;
		GetWindowRect(&rect);
		MoveWindow(rect.left, rect.top, rect.right - rect.left - 290, rect.bottom - rect.top);
	}
}
