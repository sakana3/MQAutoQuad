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

int IntersectLineAndLine( const MQPoint &a, const MQPoint &b, const MQPoint &c, const MQPoint &d)
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
	friend class MQAutoQuadWindow;

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

	std::pair< std::vector<int>, std::vector<int> > FindQuad(MQDocument doc, MQScene scene, const MQPoint& mouse_pos);

	int AddFace(MQScene scene, MQObject obj, std::vector<int> verts, int iMaterial);

	std::vector<int> FindMirror(MQObject obj, const std::vector<MQPoint>& verts, const std::vector<int>& face, float SymmetryDistance);

private:
	bool m_bActivated;
	bool m_bUseHandle;
	bool m_bShowHandle;

	std::vector<int> Quad;
	std::vector<int> Mirror;
	MQBorders borders;

	HCURSOR m_MoveCursor;
};


//---------------------------------------------------------------------------
//  SingleMovePlugin::SingleMovePlugin
//    コンストラクタ
//---------------------------------------------------------------------------
SingleMovePlugin::SingleMovePlugin()
{
	m_bActivated = false;
	m_MoveCursor = NULL;
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
}

//---------------------------------------------------------------------------
//  SingleMovePlugin::Activate
//    表示・非表示切り替え要求
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::Activate(MQDocument doc, BOOL flag)
{
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

	if ( Quad.size() == 4 )
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

		if (Quad.size() == Mirror.size())
		{
			int mirrorface[4];
			mirrorface[0] = drawQuad->AddVertex(obj->GetVertex(Mirror[0]));
			mirrorface[1] = drawQuad->AddVertex(obj->GetVertex(Mirror[1]));
			mirrorface[2] = drawQuad->AddVertex(obj->GetVertex(Mirror[2]));
			mirrorface[3] = drawQuad->AddVertex(obj->GetVertex(Mirror[3]));
			auto iMirror = drawQuad->AddFace(4, mirrorface);
			drawQuad->SetFaceMaterial(iMirror, iMaterial);

			int revmirror[4] = { mirrorface[3] , mirrorface[2] , mirrorface[1] , mirrorface[0] };
			iMirror = drawQuad->AddFace(4, revmirror);
			drawQuad->SetFaceMaterial(iMirror, iMaterial);
		}
	}
}

//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonDown
//    左ボタンが押されたとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnLeftButtonDown(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	if (Quad.size() == 4)
	{
		MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());

		auto face = AddFace(scene, obj, Quad, doc->GetCurrentMaterialIndex());
		auto mirror = -1;
		if (Quad.size() == Mirror.size())
		{
			mirror = AddFace(scene, obj, Mirror, doc->GetCurrentMaterialIndex());
		}

		if (!IsFrontFace(scene, obj, face))
		{
			obj->InvertFace(face);
			if (mirror >= 0)
			{
				obj->InvertFace(mirror);
			}
		}

		obj->Compact();

		RedrawAllScene();
		UpdateUndo(L"Auto Quad");
		return TRUE;
	}

	// It returns TRUE because an original action was done. A default action will be ignored.
	// 標準動作の代わりに独自処理を行ったのでTRUEを返す
	return FALSE;
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonMove
//    左ボタンが押されながらマウスが移動したとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnLeftButtonMove(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	return FALSE;
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonUp
//    左ボタンが離されたとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnLeftButtonUp(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	return FALSE;
}



//---------------------------------------------------------------------------
//  SingleMovePlugin::OnMouseMove
//    マウスが移動したとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnMouseMove(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	bool redraw = false;

	auto mouse_pos = MQPoint((float)state.MousePos.x, (float)state.MousePos.y, 0);
	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());

	HIT_TEST_PARAM param;
	std::vector<MQObject> objlist;
	objlist.push_back(obj);
	this->HitTestObjects(scene, state.MousePos , objlist, param);

	std::vector<int> new_quad;
	std::vector<int> new_mirror;
	if (param.HitType != MQCommandPlugin::HIT_TYPE::HIT_TYPE_FACE)
	{
		auto quads = FindQuad(doc, scene, mouse_pos);
		new_quad = quads.first;
		new_mirror = quads.second;
	}

	if (new_quad != Quad)
	{
		Quad = new_quad;
		Mirror = new_mirror;
		redraw = true;
	}

/*
	if(m_ActiveHandle == MQHandleInfo::HandleNone && m_iHighlightVertex != -1){
		SetMouseCursor(m_MoveCursor);
	}else{
		SetMouseCursor(GetResourceCursor(MQCURSOR_DEFAULT));
	}
*/
	if(redraw){
		RedrawScene(scene);
	}

	// It returns FALSE for a default action.
	// 独自処理を行ったが、標準動作も行わせるためFALSEを返す
	return FALSE;
}

std::pair< std::vector<int> , std::vector<int> > SingleMovePlugin::FindQuad(MQDocument doc, MQScene scene, const MQPoint& mouse_pos)
{
	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());

	// ボーダーエッジ抽出
	borders = MQBorders(scene, obj);

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

	std::vector<int> mirror;
	if (new_quad.size() != 4)
	{
		new_quad.clear();
	}
	else
	{
		EDIT_OPTION option;
		GetEditOption(option);
		if (option.Symmetry)
		{
			mirror = FindMirror(obj, verts, new_quad, option.SymmetryDistance);
		}
	}

	return std::pair< std::vector<int>, std::vector<int> >( new_quad , mirror );
}


int SingleMovePlugin::AddFace(MQScene scene, MQObject obj,  std::vector<int> verts, int iMaterial)
{
	auto face = obj->AddFace(4, verts.data());
	obj->SetFaceMaterial(face, iMaterial);

	for (int fi = 0; fi < obj->GetFaceCount(); fi++)
	{
		auto cnt = obj->GetFacePointCount(fi);
		// 不要になったエッジを削除する。（これ必要？）
		if (cnt == 2)
		{
			int v[2];
			obj->GetFacePointArray(fi, v);
			for (int ie = 0; ie < verts.size(); ie++)
			{
				int e0 = verts[ie];
				int e1 = verts[(ie + 1) % 4];
				if ((e0 == v[0] && e1 == v[1]) || (e0 == v[1] && e1 == v[0]))
				{
					obj->DeleteFace(fi, false);
				}
			}
		}
	}


	return face;
}

std::vector<int> SingleMovePlugin::FindMirror(MQObject obj, const std::vector<MQPoint>& verts, const std::vector<int>& poly, float SymmetryDistance)
{
	std::vector<int> mirror(4, -1);
	auto dist = SymmetryDistance * SymmetryDistance;
	std::vector<MQPoint> mirrorPos(poly.size());
	for (int i = 0; i < poly.size(); i++)
	{
		auto p = verts[poly[i]];
		mirrorPos[i] = MQPoint(-p.x, p.y, p.z);
	}

	for (int fi = 0; fi < poly.size(); fi++)
	{
		auto p = mirrorPos[fi];
		auto dist = SymmetryDistance * SymmetryDistance;
		for (int vi = 0; vi < verts.size(); vi++)
		{
			auto v = verts[vi];
			auto d = v - p;
			auto f = d.norm();
			if (dist >= f)
			{
				mirror[fi] = vi;
				dist = f;
			}
		}
	}

	if (std::find(mirror.begin(), mirror.end(), -1) != mirror.end() )
	{
		mirror.clear();
	}
	else if (mirror == poly)
	{
		mirror.clear();
	}
	else
	{
		std::reverse(mirror.begin(), mirror.end());
	}

	return mirror;
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

