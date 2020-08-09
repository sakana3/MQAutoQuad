#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <float.h>
#include "MQBasePlugin.h"
#include "MQHandleObject.h"
#include "MQSelectOperation.h"
#include "MQWidget.h"
#include "MQ3DLib.h"
#include "resource.h"
#include <limits>
#include <algorithm>
#include <iostream>

HINSTANCE g_hInstance;


#define Trace( str, ... ) \
      { \
        TCHAR c[4096]; \
        sprintf_s( c , 4096, str, __VA_ARGS__ ); \
        OutputDebugString( c ); \
      }


class MQBorders
{
public:
	typedef std::pair<int, int> tedge;
	std::vector<tedge> edges;
	std::vector<int> verts;

	MQBorders()
	{

	}

	MQBorders( const MQBorders& border)
	{
		edges = border.edges;
		verts = border.verts;
	}

	MQBorders(MQScene scene , MQObject obj )
	{
		//フェイスのエッジのリストアップ
		auto fcnt = obj->GetFaceCount();
		std::map<tedge,int> edgemap;
		for(int fi = 0; fi < fcnt ; fi++)
		{
			auto pcnt = obj->GetFacePointCount(fi);
			int* points = (int*)alloca(sizeof(int) * pcnt);
			obj->GetFacePointArray(fi, points);
			if (pcnt == 2)
			{
				auto e0 = points[0];
				auto e1 = points[1];
				tedge edge = (e0 > e1) ? tedge(e1, e0) : tedge(e0, e1);
				edgemap[edge] = true;
			}
			else if( pcnt > 2 )
			{
				for (int ie = 0; ie < pcnt; ie++)
				{
					if (IsFrontFace(scene, obj, fi))
					{
						auto e0 = points[ie];
						auto e1 = points[(ie + 1) % pcnt];
						tedge edge = (e0 > e1) ? tedge(e1, e0) : tedge(e0, e1);

						if (edgemap.find(edge) == edgemap.end())
						{
							edgemap[edge] = true;
						}
						else
						{
							edgemap[edge] = false;
						}
					}
				}
			}
		}
		// ボーダーエッジを抽出
		edges.clear();
		edges.reserve(edgemap.size());
		for (const auto& edge : edgemap)
		{
			if (edge.second)
			{
				edges.push_back(edge.first);
			}
		}

		//ボーダー頂点の抽出
		std::set<int> vertset;
		for (const auto& edge : edges)
		{
			vertset.insert(edge.first);
			vertset.insert(edge.second);
		}
		verts.clear();
		verts.reserve(vertset.size());
		for (const int vert : vertset)
		{
			verts.push_back(vert);
		}
	}
};

double LEN( const MQPoint2& a, const MQPoint2& b)
{
	double dx = (a.x - b.x);
	double dy = (a.y - b.y);
	return (dx * dx) + (dy * dy);
}

MQPoint2 CENTER(const MQPoint2& a, const MQPoint2& b , const MQPoint2& c )
{
	double dx = (a.x - b.x);
	double dy = (a.y - b.y);
	return (a + b + c) / 3.0f;
}

std::vector<int>  MakeQuad(const std::vector<int>& quad, const std::vector< MQPoint >& points)
{
	std::vector<int> temp(quad);
	std::sort(temp.begin(), temp.end());

	int q0 = temp[0];
	int q1 = temp[1];
	int q2 = temp[2];
	int q3 = temp[3];

	auto n012 = GetNormal(points[q0], points[q1], points[q2]);
	auto n230 = GetNormal(points[q2], points[q3], points[q0]);
	auto n021 = GetNormal(points[q0], points[q2], points[q1]);
	auto n320 = GetNormal(points[q3], points[q2], points[q0]);

	std::vector<int> result(4);
	if (GetInnerProduct(n012, n230) > GetInnerProduct(n012, n320) )
	{
		result[0] = q0;
		result[1] = q1;
		result[2] = q2;
		result[3] = q3;
	}
	else
	{
		result[0] = q0;
		result[1] = q1;
		result[2] = q3;
		result[3] = q2;
	}

	return result;
}

MQPoint2 CalcProjection(const MQPoint& qp, const MQMatrix& m)
{
	float x = qp.x*m._11 + qp.y*m._21 + qp.z*m._31 + m._41;
	float y = qp.x*m._12 + qp.y*m._22 + qp.z*m._32 + m._42;
	float z = qp.x*m._13 + qp.y*m._23 + qp.z*m._33 + m._43;
	float w = qp.x*m._14 + qp.y*m._24 + qp.z*m._34 + m._43;

	if (w > 0)
	{
		x = x / w;
		y = y / w;
		z = z / w;

		return MQPoint2(x, y);
	}

	return MQPoint2( -0 , -0 );
}

int IntersectLineAndLine( MQPoint &a, MQPoint &b, MQPoint &c, MQPoint &d)
{
	float s, t;
	s = (a.x - b.x) * (c.y - a.y) - (a.y - b.y) * (c.x - a.x);
	t = (a.x - b.x) * (d.y - a.y) - (a.y - b.y) * (d.x - a.x);
	if (s * t > 0)
		return false;

	s = (c.x - d.x) * (a.y - c.y) - (c.y - d.y) * (a.x - c.x);
	t = (c.x - d.x) * (b.y - c.y) - (c.y - d.y) * (b.x - c.x);
	if (s * t > 0)
		return false;
	return true;
}


bool PointInTriangle( const MQPoint& p , const MQPoint& v1 , const MQPoint& v2 , const MQPoint& v3 )
{
	#define CROSS(p1,p2,p3) ((p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y))
	bool b1 = CROSS(p, v1, v2) < 0.0f;
	bool b2 = CROSS(p, v2, v3) < 0.0f;
	bool b3 = CROSS(p, v3, v1) < 0.0f;
	return ((b1 == b2) && (b2 == b3));
}

bool PointInQuad(const MQPoint& p, const MQPoint& v0, const MQPoint& v1, const MQPoint& v2, const MQPoint& v3)
{
	auto r0 = PointInTriangle(p, v0, v1, v2);
	if (r0) return true;
	auto r1 = PointInTriangle(p, v2, v3, v0);
	return r1;
}


class SingleMovePlugin : public MQCommandPlugin
{
	friend class SingleMoveWindow;

public:
	// コンストラクタ
	SingleMovePlugin();

	// プラグインIDを返す。
	virtual void GetPlugInID(DWORD *Product, DWORD *ID);
	// プラグイン名を返す。
	virtual const char *GetPlugInName(void);
	// ポップアップメニューに表示される文字列を返す。
	virtual const wchar_t *EnumString(void);
	// アプリケーションの初期化
	virtual BOOL Initialize();
	// アプリケーションの終了
	virtual void Exit();
	// 表示・非表示切り替え要求
	virtual BOOL Activate(MQDocument doc, BOOL flag);
	// 描画時の処理
	virtual void OnDraw(MQDocument doc, MQScene scene, int width, int height);

	// 左ボタンが押されたとき
	virtual BOOL OnLeftButtonDown(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state);
	// 左ボタンが押されながらマウスが移動したとき
	virtual BOOL OnLeftButtonMove(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state);
	// 左ボタンが離されたとき
	virtual BOOL OnLeftButtonUp(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state);
	// マウスが移動したとき
	virtual BOOL OnMouseMove(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state);

private:
	bool m_bUseHandle;

	bool m_bActivated;
	int m_iHighlightObject;
	int m_iHighlightVertex;
	int m_iMoveObject;
	int m_iMoveVertex;
	bool m_bMoved;
	bool m_bShowHandle;
	MQHandleInfo::AxisType m_ActiveHandle;
	MQHandleInfo::AxisType m_MoveHandle;
	MQHandleOperation m_HandleOperation;
	MQPoint m_HandleOffset;
	MQPoint m_ScreenPos;
	MQPoint m_MovedPos;
	MQSelectOperation m_SelectOperation;

	int Quad[4] = {-1,-1,-1,-1};
	MQBorders borders;

	HCURSOR m_MoveCursor;

	SingleMoveWindow *m_Window;
};


class SingleMoveWindow : public MQWindow
{
public:
	SingleMoveWindow(int id, SingleMovePlugin *plugin);
	~SingleMoveWindow();

	MQCheckBox *m_UseHandleCheck;

	SingleMovePlugin *m_Plugin;

	BOOL UseHandleChanged(MQWidgetBase *sender, MQDocument doc);

private:

};


SingleMoveWindow::SingleMoveWindow(int id, SingleMovePlugin *plugin) : MQWindow(id)
{
	m_Plugin = plugin;

	MQFrame *frame = CreateVerticalFrame(this);
	frame->SetOutSpace(0.2);

	MQFrame *hframe = CreateHorizontalFrame(frame);
	hframe->SetUniformSize(true);

	m_UseHandleCheck = CreateCheckBox(frame, L"Use handle");
	m_UseHandleCheck->SetChecked(m_Plugin->m_bUseHandle);
	m_UseHandleCheck->AddChangedEvent(this, &SingleMoveWindow::UseHandleChanged);
}

SingleMoveWindow::~SingleMoveWindow()
{
	//UnregisterDisabledWidget(m_ClosedCheck);
}

BOOL SingleMoveWindow::UseHandleChanged(MQWidgetBase *sender, MQDocument doc)
{
	m_Plugin->m_bUseHandle = m_UseHandleCheck->GetChecked();

	if(m_Plugin->m_bShowHandle){
		m_Plugin->m_bShowHandle = false;
		m_Plugin->RedrawAllScene();
	}
	return TRUE;
}



//---------------------------------------------------------------------------
//  SingleMovePlugin::SingleMovePlugin
//    コンストラクタ
//---------------------------------------------------------------------------
SingleMovePlugin::SingleMovePlugin()
{
	m_bUseHandle = true;

	m_bActivated = false;
	m_iHighlightObject = -1;
	m_iHighlightVertex = -1;
	m_iMoveObject = -1;
	m_iMoveVertex = -1;
	m_bMoved = false;
	m_bShowHandle = false;
	m_ActiveHandle = MQHandleInfo::HandleNone;
	m_MoveHandle = MQHandleInfo::HandleNone;

	m_MoveCursor = NULL;

	m_Window = NULL;
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::GetPlugInID
//    プラグインIDを返す。
//---------------------------------------------------------------------------
void SingleMovePlugin::GetPlugInID(DWORD *Product, DWORD *ID)
{
	// プロダクト名(制作者名)とIDを、全部で64bitの値として返す
	// 値は他と重複しないようなランダムなもので良い
	*Product = 0xA0E090AD;
	*ID      = 0x444490AD;
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::GetPlugInName
//    プラグイン名を返す。
//---------------------------------------------------------------------------
const char *SingleMovePlugin::GetPlugInName(void)
{
	return "Auto Quad      by sakana3";
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::EnumString
//    ボタンに表示される文字列を返す。
//---------------------------------------------------------------------------
const wchar_t *SingleMovePlugin::EnumString(void)
{
	return L"Auto Quad";
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::Initialize
//    アプリケーションの初期化
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::Initialize()
{
	// Load a cursor.
	// カーソルをロード
	if(m_MoveCursor == NULL){
		m_MoveCursor = LoadCursor(g_hInstance, MAKEINTRESOURCE(IDC_CURSOR1));
	}

	return TRUE;
}

//---------------------------------------------------------------------------
//  SingleMovePlugin::Exit
//    アプリケーションの終了
//---------------------------------------------------------------------------
void SingleMovePlugin::Exit()
{
	// Release the window
	// ウィンドウを破棄
	delete m_Window;
	m_Window = NULL;

}

//---------------------------------------------------------------------------
//  SingleMovePlugin::Activate
//    表示・非表示切り替え要求
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::Activate(MQDocument doc, BOOL flag)
{
	if (flag){
		// Create a window
		// ウィンドウを作成
		m_Window = new SingleMoveWindow(MQWindow::GetSystemWidgetID(MQSystemWidget::OptionPanel), this);
		m_Window->SetTitle(L"SingleMove");
		POINT size = m_Window->GetJustSize();
		m_Window->SetWidth(size.x);
		m_Window->SetHeight(size.y);
		m_Window->SetVisible(true);
	}else{
		// Delete a window
		// ウィンドウを破棄
		delete m_Window;
		m_Window = NULL;

		// Erase a highlight
		// ハイライト表示を消す
		if(m_bShowHandle || (m_iHighlightObject != -1 && m_iHighlightVertex != -1)){
			m_bShowHandle = false;
			m_iHighlightObject = -1;
			m_iHighlightVertex = -1;
			RedrawAllScene();
		}
	}

	m_bActivated = flag ? true : false;
	
	// It returns 'flag' as it is.
	// そのままflagを返す
	return flag;
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::OnDraw
//    描画時の処理
//---------------------------------------------------------------------------
void SingleMovePlugin::OnDraw(MQDocument doc, MQScene scene, int width, int height)
{
	if(!m_bActivated) return;

	// Show a handle.
	// ハンドルを表示
	if (m_iMoveObject != -1 && m_iMoveVertex != -1 && m_bShowHandle)
	{
		MQObject obj = doc->GetObject(m_iMoveObject);
		if(obj != NULL && obj->GetVertexRefCount(m_iMoveVertex) > 0){
			MQPoint pos = obj->GetVertex(m_iMoveVertex);

			MQHandleMaterial handle_mat;
			MQHandleInfo info(scene, pos);
			info.active = (MQHandleInfo::AxisType)m_ActiveHandle;
			MQHandleObject handle;
			handle.CreateMoveHandle(this, true, doc, info, &handle_mat);		
		}
	}

	// Highlight the vertex.
	// 頂点をハイライト描画
	if (m_iHighlightObject != -1 && m_iHighlightVertex != -1)
	{
		MQObject obj = doc->GetObject(m_iHighlightObject);
		if (obj == NULL) return; // あり得ないはずだが、念のためNULLチェック

		// Calculate the position of the vertex.
		// 頂点位置を計算
		MQPoint sp = scene->Convert3DToScreen(obj->GetVertex(m_iHighlightVertex));
		MQPoint newvp = scene->ConvertScreenTo3D(MQPoint(sp.x,sp.y,scene->GetFrontZ()));

		// Create a drawing object.
		// 描画オブジェクトを生成
		int vertex[1];
		MQObject draw = CreateDrawingObject(doc, DRAW_OBJECT_POINT);
		draw->SetColor(MQColor(1,1,0));
		draw->SetColorValid(TRUE);
		vertex[0] = draw->AddVertex(newvp);
		draw->AddFace(1, vertex);

		for (auto q : Quad)
		{
			if (q >= 0)
			{
				draw->AddVertex(obj->GetVertex(q));
			}
		}
	}


	MQObject drawEdge = CreateDrawingObject(doc, DRAW_OBJECT_LINE);

	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());
	for ( const auto& edge : borders.edges)
	{
		MQPoint p0 = obj->GetVertex( edge.first );
		MQPoint p1 = obj->GetVertex( edge.second );

		drawEdge->SetColor(MQColor(1, 1, 0));
		drawEdge->SetColorValid(TRUE);

		int newface[2];
		newface[0] = drawEdge->AddVertex(p0);
		newface[1] = drawEdge->AddVertex(p1);
		drawEdge->AddFace(2, newface);
	}

	if (Quad[0] >= 0 && Quad[1] >= 0 && Quad[2] >= 0 && Quad[3] >= 0)
	{
		MQObject drawQuad = CreateDrawingObject(doc, DRAW_OBJECT_FACE );
		int iMaterial;
		auto mat = CreateDrawingMaterial(doc, iMaterial);
		mat->SetShader(MQMATERIAL_SHADER_CONSTANT);
		mat->SetColor(MQColor(1, 1, 0));
		mat->SetAlpha(0.25f);

		drawQuad->SetColor(MQColor(1, 1, 0));
		drawQuad->SetColorValid(TRUE);

		int newface[4];
		newface[0] = drawQuad->AddVertex( obj->GetVertex(Quad[0]) );
		newface[1] = drawQuad->AddVertex( obj->GetVertex(Quad[1]) );
		newface[2] = drawQuad->AddVertex( obj->GetVertex(Quad[2]) );
		newface[3] = drawQuad->AddVertex( obj->GetVertex(Quad[3]) );
		auto iFace = drawQuad->AddFace(4, newface);
		drawQuad->SetFaceMaterial(iFace, iMaterial);

		int revface[4] = { newface[3] , newface[2] , newface[1] , newface[0] };
		iFace = drawQuad->AddFace(4, revface);
		drawQuad->SetFaceMaterial(iFace, iMaterial);
	}

	if(m_SelectOperation.GetType() != MQSelectOperation::SELECT_NONE)
	{
		m_SelectOperation.Draw(this, doc, scene);
	}
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonDown
//    左ボタンが押されたとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnLeftButtonDown(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	MQPoint clickpos((float)state.MousePos.x, (float)state.MousePos.y, 0);
	float mindis2 = 10*10; //10ピクセル以内

	// It gets edit options.
	// 編集オプションを取得
	EDIT_OPTION option;
	GetEditOption(option);

	if(m_bShowHandle)
	{
		MQObject obj = doc->GetObject(m_iMoveObject);
		if(obj != NULL && obj->GetVertexRefCount(m_iMoveVertex) > 0){
			MQPoint center = obj->GetVertex(m_iMoveVertex);
			MQHandleInfo info(scene, center);
			MQPoint hit_pos;
			m_MoveHandle = MQHandleObject::HitTestMoveHandle(this, doc, scene, state, info, hit_pos);
			if(m_MoveHandle != MQHandleInfo::HandleNone){
				m_HandleOperation.Begin(this, info, m_MoveHandle, state.MousePos, hit_pos);
				m_HandleOffset = MQPoint(0,0,0);
				return TRUE;
			}
		}
	}

	m_iMoveObject = -1;
	m_iMoveVertex = -1;
	m_bShowHandle = false;
	m_MoveHandle = MQHandleInfo::HandleNone;
	m_bMoved = false;
	m_SelectOperation.Clear();

	// Find a nearest vertex from the clicked position.
	// クリックされた位置に最も近い頂点を見つける
	int numObj = doc->GetObjectCount();
	for (int oi=0; oi<numObj; oi++)
	{
		MQObject obj = doc->GetObject(oi);
		if (obj == NULL) continue;

		// An invisible or locked object is not moved.
		// 非表示またはロックされていれば対象外
		if (!obj->GetVisible() || obj->GetLocking()) continue;

		// It edits a current object only.
		// カレントオブジェクトのみ編集
		if (option.CurrentObjectOnly && oi != doc->GetCurrentObjectIndex())
			continue;

		int numVertex = obj->GetVertexCount();
		for (int vi=0; vi<numVertex; vi++)
		{
			if (obj->GetVertexRefCount(vi) != 0)
			{
				// Is the vertex closed to the clicked position?
				// クリックされた位置に近いかどうか
				MQPoint sp = scene->Convert3DToScreen(obj->GetVertex(vi));
				float dis2 = (sp.x-clickpos.x)*(sp.x-clickpos.x) + (sp.y-clickpos.y)*(sp.y-clickpos.y);
				if (mindis2 > dis2){
					// It will move this vertex.
					// この頂点を移動対象に
					mindis2 = dis2;
					m_iMoveObject = oi;
					m_iMoveVertex = vi;
					m_ScreenPos = sp;
				}
			}
		}
	}

	if(m_iMoveObject == -1){
		if(option.SelectRect){
			m_SelectOperation.Begin(MQSelectOperation::SELECT_RECT, state.MousePos);
		}
		if(option.SelectRope){
			m_SelectOperation.Begin(MQSelectOperation::SELECT_ROPE, state.MousePos);
		}
	}

	if (Quad[0] > -1 && Quad[1] > -1 && Quad[2] > -1 && Quad[3] > -1)
	{
		MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());

		auto face = obj->AddFace( 4 , Quad );
		obj->SetFaceMaterial(face, doc->GetCurrentMaterialIndex());

		if (!IsFrontFace(scene, obj, face))
		{
			obj->InvertFace(face);
		}

		for (int fi = 0; fi < obj->GetFaceCount(); fi++)
		{
			auto cnt = obj->GetFacePointCount(fi);
			// 不要になったエッジを削除する。（これ必要？）
			if (cnt == 2)
			{
				int verts[2];
				obj->GetFacePointArray(fi, verts);
				for (int ie = 0; ie < 4; ie++)
				{
					int e0 = Quad[ie];
					int e1 = Quad[(ie+1)%4];
					if ((e0 == verts[0] && e1 == verts[1]) || (e0 == verts[1] && e1 == verts[0]))
					{
						obj->DeleteFace(fi, false);
					}
				}
			}
		}


		obj->Compact();

		RedrawAllScene();
		UpdateUndo(L"Auto Quad");
	}

	// It returns TRUE because an original action was done. A default action will be ignored.
	// 標準動作の代わりに独自処理を行ったのでTRUEを返す
	return TRUE;
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonMove
//    左ボタンが押されながらマウスが移動したとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnLeftButtonMove(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	if (m_iMoveObject != -1 && m_iMoveVertex != -1)
	{
		MQObject obj = doc->GetObject(m_iMoveObject);
		if (obj == NULL) return TRUE; // あり得ないはずだが、念のためNULLチェック

		// 編集オプションを取得
		EDIT_OPTION option;
		GetEditOption(option);

		MQPoint vp;
		if(m_MoveHandle != MQHandleInfo::HandleNone){
			MQPoint offset = m_HandleOffset;
			m_HandleOperation.Move(offset, scene, state.MousePos);
			
			MQPoint np = obj->GetVertex(m_iMoveVertex) + offset;
			GET_SNAP_PARAM snap_param;
			snap_param.IgnoreSelected = TRUE;
			snap_param.SnapEyeDir = (m_MoveHandle == MQHandleInfo::HandleW);
			vp = GetSnappedPos(scene, np, option, snap_param); // グリッドへの吸着
			m_HandleOffset = np - vp;
			obj->SetVertex(m_iMoveVertex, vp);
		}else{
			// 頂点を移動（スクリーン座標のZはそのままでX/Yのみ動かす）
			vp = scene->ConvertScreenTo3D(MQPoint((float)state.MousePos.x, (float)state.MousePos.y, m_ScreenPos.z));
			GET_SNAP_PARAM snap_param;
			snap_param.IgnoreSelected = TRUE;
			snap_param.SnapEyeDir = TRUE;
			vp = GetSnappedPos(scene, vp, option, snap_param); // グリッドへの吸着
			obj->SetVertex(m_iMoveVertex, vp);
		}

		// 再描画
		RedrawScene(scene);

		// 移動済みチェック
		m_bMoved = true;
		m_MovedPos = vp;
	}
	else if(m_SelectOperation.GetType() != MQSelectOperation::SELECT_NONE)
	{
		m_SelectOperation.Move(state.MousePos);
	}

	// It returns TRUE because an original action was done. A default action will be ignored.
	// 標準動作の代わりに独自処理を行ったのでTRUEを返す
	return TRUE;
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonUp
//    左ボタンが離されたとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnLeftButtonUp(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	if (m_iMoveObject != -1 && m_iMoveVertex != -1)
	{
		if(m_bMoved)
		{
			// Redraw a scene and update an undo if a vertex was moved.
			// 頂点が移動された場合、再描画とアンドゥの更新を行う
			RedrawAllScene();
			UpdateUndo(L"SingleMove");

			// Send a message to StationSpy plug-in.
			// StationSpyプラグインにメッセージを送る
			char buf[128];
			sprintf_s(buf, sizeof(buf), "Moved to (%.3f %.3f %.3f)", m_MovedPos.x, m_MovedPos.y, m_MovedPos.z);
			int ret = SendUserMessage(doc, 0x56A31D20, 0x9CE001E3, "text", buf);
			sprintf_s(buf, sizeof(buf), "Received %d from SendUserMessage()\n", ret);
			OutputDebugString(buf);
		}
		else
		{
			// Show a handle.
			// ハンドルを表示
			if(m_bUseHandle){
				m_bShowHandle = true;
				RedrawAllScene();
			}
		}
	}
	else if(m_SelectOperation.GetType() != MQSelectOperation::SELECT_NONE)
	{
		// Select vertices. (The selection does not affect to the movement, just a sample)
		// 頂点を選択（移動操作には影響しないが、単なるサンプル）
		int obj_index = doc->GetCurrentObjectIndex();
		MQObject obj = doc->GetObject(obj_index);
		if(obj != NULL){
			int num = obj->GetVertexCount();
			for(int i=0; i<num; i++){
				if(obj->GetVertexRefCount(i) > 0){
					MQPoint p = scene->Convert3DToScreen(obj->GetVertex(i));
					if(p != MQPoint(0,0,0)){
						if(m_SelectOperation.IsInside(p)){
							doc->AddSelectVertex(obj_index, i);
						}else{
							doc->DeleteSelectVertex(obj_index, i);
						}
					}
				}
			}
		}
		m_SelectOperation.Finish();
		RedrawAllScene();
	}

	// It returns TRUE because an original action was done. A default action will be ignored.
	// 標準動作の代わりに独自処理を行ったのでTRUEを返す
	return TRUE;
}



//---------------------------------------------------------------------------
//  SingleMovePlugin::OnMouseMove
//    マウスが移動したとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnMouseMove(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	float mindis2 = 10 * 10; //10ピクセル以内

	int newoi = -1;
	int newvi = -1;
	bool redraw = false;

	// It gets edit options.
	// 編集オプションを取得
	EDIT_OPTION option;
	GetEditOption(option);

	MQHandleInfo::AxisType active_handle = MQHandleInfo::HandleNone;
	if (m_bShowHandle)
	{
		MQObject obj = doc->GetObject(m_iMoveObject);
		if (obj != NULL && obj->GetVertexRefCount(m_iMoveVertex) > 0) {
			MQPoint center = obj->GetVertex(m_iMoveVertex);
			MQHandleInfo info(scene, center);
			MQPoint hit_pos;
			active_handle = MQHandleObject::HitTestMoveHandle(this, doc, scene, state, info, hit_pos);
		}
	}
	if (m_ActiveHandle != active_handle) {
		m_ActiveHandle = active_handle;
		redraw = true;
	}

	if (m_ActiveHandle == MQHandleInfo::HandleNone)
	{
		// Find a nearest vertex from the mouse cursor.
		// カーソル位置に最も近い頂点を見つける
		int numObj = doc->GetObjectCount();
		for (int oi = 0; oi < numObj; oi++)
		{
			MQObject obj = doc->GetObject(oi);
			if (obj == NULL) continue;

			// An invisible or locked object is not moved.
			// 非表示またはロックされていれば対象外
			if (!obj->GetVisible() || obj->GetLocking()) continue;

			// It edits a current object only.
			// カレントオブジェクトのみ編集
			if (option.CurrentObjectOnly && oi != doc->GetCurrentObjectIndex())
				continue;

			int numVertex = obj->GetVertexCount();
			for (int vi = 0; vi < numVertex; vi++)
			{
				if (obj->GetVertexRefCount(vi) != 0)
				{
					// Is the vertex closed to the cursor position?
					// カーソル位置に近いかどうか
					MQPoint sp = scene->Convert3DToScreen(obj->GetVertex(vi));
					float dis2 = (sp.x - state.MousePos.x)*(sp.x - state.MousePos.x) + (sp.y - state.MousePos.y)*(sp.y - state.MousePos.y);
					if (mindis2 > dis2) {
						// It will move this vertex.
						// この頂点を移動対象に
						mindis2 = dis2;
						newoi = oi;
						newvi = vi;
					}
				}
			}
		}
	}

	// Redraw a scene when the cursor is on the different vertex from the previous time.
	// 前回と違う頂点上にカーソルがきた場合は再描画
	if (m_iHighlightObject != newoi || m_iHighlightVertex != newvi)
	{
		m_iHighlightObject = newoi;
		m_iHighlightVertex = newvi;
		redraw = true;
	}

	auto mouse_pos = MQPoint((float)state.MousePos.x, (float)state.MousePos.y, 0);
	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());

	HIT_TEST_PARAM param;
	std::vector<MQObject> objlist;
	objlist.push_back(obj);
	this->HitTestObjects(scene, state.MousePos , objlist, param);

	if (param.HitType != MQCommandPlugin::HIT_TYPE::HIT_TYPE_FACE)
	{
		// ボーダーエッジ抽出
		borders = MQBorders( scene , obj);

		// 頂点の射影変換
		typedef std::pair<int, float> pair;
		int numVertex = obj->GetVertexCount();
		auto verts = std::vector<MQPoint>(numVertex);
		auto coords = std::vector<MQPoint>(numVertex);
		obj->GetVertexArray(verts.data());

		auto vertset = std::vector< pair >();
		for (int vi : borders.verts)
		{
			float w = 0.0f;
			auto coord = scene->Convert3DToScreen(verts[vi], &w);
			coords[vi] = coord;

			if (w > 0.0f)
			{
				float dx = mouse_pos.x - coords[vi].x;
				float dy = mouse_pos.y - coords[vi].y;
				float dist = dx * dx + dy * dy;
				vertset.push_back(pair(vi, dist));
			}
		}

		std::sort(
			vertset.begin(),
			vertset.end(),
			[](const pair& lhs, const pair& rhs) { return lhs.second < rhs.second; }
		);

		std::vector<int> new_quad = std::vector<int>();
		new_quad.reserve(4);
		for (const pair& t : vertset)
		{
			auto v = t.first;
			auto p = coords[v];

			bool isHitOtherEdge = false;
			for (const std::pair<int, int>& border : borders.edges)
			{
				if (IntersectLineAndLine(mouse_pos, p, coords[border.first], coords[border.second]))
				{
					if (border.first != v && border.second != v)
					{
						isHitOtherEdge = true;
						break;
					}
				}
			}
			if (!isHitOtherEdge)
			{
				new_quad.push_back(t.first);
				if (new_quad.size() >= 4)
				{
					auto tmp_quad = MakeQuad(new_quad, verts);
					if (PointInQuad(mouse_pos, coords[tmp_quad[0]], coords[tmp_quad[1]], coords[tmp_quad[2]], coords[tmp_quad[3]]))
					{
						new_quad = tmp_quad;
						break;
					}
					else
					{
						new_quad.pop_back();
					}
				}
			}
		}


		// Quad作成マン
		auto old_Quad = std::vector<int>(std::begin(Quad), std::end(Quad));
		Quad[0] = -1;
		Quad[1] = -1;
		Quad[2] = -1;
		Quad[3] = -1;

		if (new_quad.size() >= 4)
		{
			for (int i = 0; i < 4; i++)
			{
				Quad[i] = new_quad[i];
			}
		}

		if (old_Quad[0] != Quad[0] || old_Quad[1] == Quad[1] || old_Quad[1] == Quad[1] || old_Quad[1] == Quad[1])
		{
			redraw = true;
		}
	}
	else
	{
		Quad[0] = -1;
		Quad[1] = -1;
		Quad[2] = -1;
		Quad[3] = -1;
	}

	if(m_ActiveHandle == MQHandleInfo::HandleNone && m_iHighlightVertex != -1){
		SetMouseCursor(m_MoveCursor);
	}else{
		SetMouseCursor(GetResourceCursor(MQCURSOR_DEFAULT));
	}

	if(redraw){
		RedrawScene(scene);
	}

	// It returns FALSE for a default action.
	// 独自処理を行ったが、標準動作も行わせるためFALSEを返す
	return FALSE;
}



//---------------------------------------------------------------------------
//  GetPluginClass
//    プラグインのベースクラスを返す
//---------------------------------------------------------------------------
MQBasePlugin *GetPluginClass()
{
	static SingleMovePlugin plugin;
	return &plugin;
}



//---------------------------------------------------------------------------
//  DllMain
//---------------------------------------------------------------------------
BOOL APIENTRY DllMain( HINSTANCE hinstDLL, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
	if(ul_reason_for_call == DLL_PROCESS_ATTACH){
		g_hInstance = hinstDLL;
	}
    return TRUE;
}

