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


class SceneChash
{
public:
	class Scene
	{
	public:
		MQObject obj;
		std::vector<MQPoint> verts;
		std::vector<MQPoint> coords;
		std::vector<bool> in_screen;

		Scene() {}
		Scene(const Scene& screen)
		{
			obj = screen.obj;
			verts = screen.verts;
			coords = screen.coords;
			in_screen = screen.in_screen;
		}

		Scene(MQObject obj, MQScene scene)
		{
			this->obj = obj;
			int numVertex = obj->GetVertexCount();
			verts = std::vector<MQPoint>(numVertex);
			coords = std::vector<MQPoint>(numVertex);
			in_screen = std::vector<bool>(numVertex);
			obj->GetVertexArray(verts.data());
			for (int i = 0; i < numVertex; i++)
			{
				float w = 0.0f;
				coords[i] = scene->Convert3DToScreen(verts[i], &w);
				in_screen[i] = w > 0;
			}
		}

	};

	typedef std::pair<int, int> tedge;
	std::map<MQScene, Scene> scenes;
	std::vector<tedge> border_edges;
	std::vector<int> border_verts;
	MQObject obj = NULL;

	void UpdateBorder(MQScene scene, MQObject obj)
	{
		//フェイスのエッジのリストアップ
		auto fcnt = obj->GetFaceCount();
		std::map<tedge, int> edgemap;
		for (int fi = 0; fi < fcnt; fi++)
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
			else if (pcnt > 2)
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
		border_edges.clear();
		border_edges.reserve(edgemap.size());
		for (const auto& edge : edgemap)
		{
			if (edge.second)
			{
				border_edges.push_back(edge.first);
			}
		}

		//ボーダー頂点の抽出
		std::set<int> vertset;
		for (const auto& edge : border_edges)
		{
			vertset.insert(edge.first);
			vertset.insert(edge.second);
		}
		border_verts.clear();
		border_verts.reserve(vertset.size());
		for (const int vert : vertset)
		{
			border_verts.push_back(vert);
		}
	}

	const Scene& Get(MQScene scene, MQObject obj)
	{
		if (this->obj == NULL)
		{
			UpdateBorder(scene, obj);
		}
		this->obj = obj;
		if (scenes.find(scene) == scenes.end())
		{
			scenes[scene] = Scene(obj, scene);
		}
		return scenes[scene];
	}

	void Clear( bool screen_only = false)
	{
		if (!screen_only)
		{
			border_edges.clear();
			border_verts.clear();
		}
		scenes.clear();
		this->obj = NULL;
	}
};



double LEN(const MQPoint2& a, const MQPoint2& b)
{
	double dx = (a.x - b.x);
	double dy = (a.y - b.y);
	return (dx * dx) + (dy * dy);
}

MQPoint2 CENTER(const MQPoint2& a, const MQPoint2& b, const MQPoint2& c)
{
	double dx = (a.x - b.x);
	double dy = (a.y - b.y);
	return (a + b + c) / 3.0f;
}

float ANGLE(const MQPoint2& p )
{
	float angle = acos(p.x / sqrt( p.x*p.x + p.y * p.y));
	angle = angle * 180.0f / PI;
	if (p.y<0)angle = 360.0f - angle;
	return angle;
}


std::vector<int>  MakeQuad(const std::vector<int>& quad, const std::vector< MQPoint >& points, const std::vector< MQPoint >& coords , const MQPoint& pivot)
{

//	angle = acos(x / sqrt(x*x + y * y));
//	angle = angle * 180.0 / PI;
//	if (y<0)angle = 360.0 - angle;
	std::vector< std::pair< int , float > > temp;
	temp.reserve(quad.size());
	for (auto q : quad)
	{
		auto p = coords[q] - pivot;
		float angle = acos(p.x / sqrt(p.x*p.x + p.y * p.y));
		angle = angle * 180.0f / PI;
		if (p.y<0)angle = 360.0f - angle;
		temp.push_back(std::pair< int, float >(q, angle));
	}


	std::sort(
		temp.begin(),
		temp.end(),
		[](const auto& lhs, const auto& rhs) { return lhs.second > rhs.second; }
	);

	std::vector<int> ret;
	ret.reserve(quad.size());
	for ( const auto& t : temp)
	{
		ret.push_back(t.first);
	}

	return ret;
}


std::vector<int>  MakeQuad2(const std::vector<int>& quad, const std::vector< MQPoint >& points, const std::vector< MQPoint >& coords, const MQPoint& pivot)
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
	if (GetInnerProduct(n012, n230) > GetInnerProduct(n012, n320))
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

	return MQPoint2(-0, -0);
}

int IntersectLineAndLine(const MQPoint &a, const MQPoint &b, const MQPoint &c, const MQPoint &d)
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


bool PointInTriangle(const MQPoint& p, const MQPoint& v1, const MQPoint& v2, const MQPoint& v3)
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

	// アンドゥ実行時
	virtual BOOL OnUndo(MQDocument doc, int undo_state) { screenChash.Clear(); return FALSE; }
	// リドゥ実行時
	virtual BOOL OnRedo(MQDocument doc, int redo_state) { screenChash.Clear(); return FALSE; }
	// アンドゥ状態更新時
	virtual void OnUpdateUndo(MQDocument doc, int undo_state, int undo_size) { screenChash.Clear(); }
	// オブジェクトの編集時
	virtual void OnObjectModified(MQDocument doc) { screenChash.Clear(); }
	// カレントオブジェクトの変更時
	virtual void OnUpdateObjectList(MQDocument doc) { screenChash.Clear(); }
	// シーン情報の変更時
	virtual void OnUpdateScene(MQDocument doc, MQScene scene) { screenChash.Clear(true); }

private:
	bool m_bActivated;
	bool m_bUseHandle;
	bool m_bShowHandle;

	std::vector<int> Quad;
	std::vector<int> Mirror;
	SceneChash screenChash;

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
	*ID = 0x444490AD;
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
	if (m_MoveCursor == NULL) {
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
	screenChash.Clear();
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
	if (!m_bActivated) return;

	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());

	/*
	MQObject drawEdge = CreateDrawingObject(doc, DRAW_OBJECT_LINE);

	for (const auto& edge : screenChash.border_edges)
	{
		MQPoint p0 = obj->GetVertex(edge.first);
		MQPoint p1 = obj->GetVertex(edge.second);

		drawEdge->SetColor(MQColor(1, 1, 0));
		drawEdge->SetColorValid(TRUE);

		int newface[2];
		newface[0] = drawEdge->AddVertex(p0);
		newface[1] = drawEdge->AddVertex(p1);
		drawEdge->AddFace(2, newface);
	}
	*/
	if (!Quad.empty())
	{
		MQObject drawQuad = CreateDrawingObject(doc, DRAW_OBJECT_FACE);
		int iMaterial;
		auto mat = CreateDrawingMaterial(doc, iMaterial);
		mat->SetShader(MQMATERIAL_SHADER_CONSTANT);
		mat->SetColor(MQColor(1, 1, 0));
		mat->SetAlpha(0.25f);

		drawQuad->SetColor(MQColor(1, 1, 0));
		drawQuad->SetColorValid(TRUE);

		std::vector<int> newface;
		for (auto q : Quad)
		{
			newface.push_back(drawQuad->AddVertex(obj->GetVertex(q)));
		}
		auto iFace = drawQuad->AddFace((int)newface.size(), newface.data());
		drawQuad->SetFaceMaterial(iFace, iMaterial);

		std::reverse(newface.begin(), newface.end());
		iFace = drawQuad->AddFace((int)newface.size(), newface.data());
		drawQuad->SetFaceMaterial(iFace, iMaterial);

		if (Quad.size() == Mirror.size())
		{
			std::vector<int> mirrorface;
			for (auto q : Mirror)
			{
				mirrorface.push_back(drawQuad->AddVertex(obj->GetVertex(q)));
			}

			auto iFace = drawQuad->AddFace((int)mirrorface.size(), mirrorface.data());
			drawQuad->SetFaceMaterial(iFace, iMaterial);

			std::reverse(mirrorface.begin(), mirrorface.end());
			iFace = drawQuad->AddFace((int)mirrorface.size(), mirrorface.data());
			drawQuad->SetFaceMaterial(iFace, iMaterial);
		}
	}
}

//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonDown
//    左ボタンが押されたとき
//---------------------------------------------------------------------------
BOOL SingleMovePlugin::OnLeftButtonDown(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	if (Quad.size() >= 3)
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
		screenChash.Clear();
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
	this->HitTestObjects(scene, state.MousePos, objlist, param);

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
	if (redraw) {
		RedrawScene(scene);
	}

	// It returns FALSE for a default action.
	// 独自処理を行ったが、標準動作も行わせるためFALSEを返す
	return FALSE;
}



std::pair< std::vector<int>, std::vector<int> > SingleMovePlugin::FindQuad(MQDocument doc, MQScene scene, const MQPoint& mouse_pos)
{
	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());

	//スクリーン変換
	auto screen = screenChash.Get(scene, obj);

	// 頂点の射影変換
	typedef std::pair<int, float> pair;
	int numVertex = obj->GetVertexCount();
	auto vertset = std::vector< pair >();
	vertset.reserve(numVertex);
	for (int vi : screenChash.border_verts)
	{
		if (screen.in_screen[vi])
		{
			float dx = mouse_pos.x - screen.coords[vi].x;
			float dy = mouse_pos.y - screen.coords[vi].y;
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
		auto p = screen.coords[v];

		bool isHitOtherEdge = false;
		for (const std::pair<int, int>& border : screenChash.border_edges)
		{
			if (IntersectLineAndLine(mouse_pos, p, screen.coords[border.first], screen.coords[border.second]))
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
				auto tmp_quad = MakeQuad(new_quad, screen.verts, screen.coords , mouse_pos);
				if (PointInQuad(mouse_pos, screen.coords[tmp_quad[0]], screen.coords[tmp_quad[1]], screen.coords[tmp_quad[2]], screen.coords[tmp_quad[3]]))
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
		//ボーダーエッジに囲まれたトライアングルだけ許可する
		if (new_quad.size() == 3)
		{
			for (int i = 0; i < new_quad.size(); i++)
			{
				auto e0 = new_quad[i];
				auto e1 = new_quad[(i + 1) % new_quad.size()];
				std::pair<int, int> edge = (e0<e1) ? std::pair<int, int>(e0, e1) : std::pair<int, int>(e1, e0);
				if (std::find(screenChash.border_edges.begin(), screenChash.border_edges.end(), edge) == screenChash.border_edges.end())
				{
					new_quad.clear();
					break;
				}
			}
		}
		else
		{
			new_quad.clear();
		}
	}

	if (!new_quad.empty())
	{
		EDIT_OPTION option;
		GetEditOption(option);
		if (option.Symmetry)
		{
			mirror = FindMirror(obj, screen.verts, new_quad, option.SymmetryDistance);
		}
	}

	return std::pair< std::vector<int>, std::vector<int> >(new_quad, mirror);
}


int SingleMovePlugin::AddFace(MQScene scene, MQObject obj, std::vector<int> verts, int iMaterial)
{
	auto face = obj->AddFace((int)verts.size(), verts.data());
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
				int e1 = verts[(ie + 1) % verts.size()];
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
	std::vector<int> mirror(poly.size(), -1);
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

	std::set<int> mirror_set(mirror.begin(), mirror.end());
	std::set<int> poly_set(poly.begin(), poly.end());

	if (std::find(mirror.begin(), mirror.end(), -1) != mirror.end())
	{
		mirror.clear();
	}
	else if (mirror_set == poly_set)
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
BOOL APIENTRY DllMain(HINSTANCE hinstDLL,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	if (ul_reason_for_call == DLL_PROCESS_ATTACH) {
		g_hInstance = hinstDLL;
	}
	return TRUE;
}

