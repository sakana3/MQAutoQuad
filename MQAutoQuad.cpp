#pragma warning(disable:4244)
#pragma warning(disable:4267)

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

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
#include <type_traits>
#include "libacc\\bvh_tree.h"
#include "MQGeometry.h"

HINSTANCE g_hInstance;

#define Trace( str, ... ) \
      { \
        TCHAR c[4096]; \
        sprintf_s( c , 4096, str, __VA_ARGS__ ); \
        OutputDebugString( c ); \
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


class MQAutoQuad : public MQCommandPlugin
{
	friend class MQAutoQuadWindow;

public:
	// コンストラクタ
	MQAutoQuad();

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

	void clear(bool isGeom, bool isScene, bool isSnap)
	{
		if (isGeom) { mqGeom.Clear(); }
		if (isScene || isGeom ) sceneCache.Clear();
		if (isSnap) mqSnap = MQSnap();
		if (isGeom || isScene) border.Clear();
#if _DEBUG
		unk3.clear();
#endif
	}

	// アンドゥ実行時
	virtual BOOL OnUndo(MQDocument doc, int undo_state) { clear(true,true,true); return FALSE; }
	// リドゥ実行時
	virtual BOOL OnRedo(MQDocument doc, int redo_state) { clear(true, true, true); return FALSE; }
	// アンドゥ状態更新時
	virtual void OnUpdateUndo(MQDocument doc, int undo_state, int undo_size) { clear(true, true, true); }
	// オブジェクトの編集時
	virtual void OnObjectModified(MQDocument doc) { clear(true, true, false); }
	// カレントオブジェクトの変更時
	virtual void OnUpdateObjectList(MQDocument doc) { clear(true, true, true); }
	// シーン情報の変更時
	virtual void OnUpdateScene(MQDocument doc, MQScene scene) { clear(false, true, false); }

private:
	bool m_bActivated;
	bool m_bUseHandle;
	bool m_bShowHandle;

	std::vector<int> Quad;
	std::vector<int> Mirror;
	MQGeom mqGeom;
	MQSnap mqSnap;
	MQSceneCache sceneCache;
	MQBorderComponent border;

#if _DEBUG
	std::vector<MQPoint> unk3;
#endif
};


//---------------------------------------------------------------------------
//  SingleMovePlugin::SingleMovePlugin
//    コンストラクタ
//---------------------------------------------------------------------------
MQAutoQuad::MQAutoQuad()
{
	m_bActivated = false;
}

//---------------------------------------------------------------------------
//  SingleMovePlugin::GetPlugInID
//    プラグインIDを返す。
//---------------------------------------------------------------------------
void MQAutoQuad::GetPlugInID(DWORD *Product, DWORD *ID)
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
const char *MQAutoQuad::GetPlugInName(void)
{
	return "Auto Quad      by sakana3";
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::EnumString
//    ボタンに表示される文字列を返す。
//---------------------------------------------------------------------------
const wchar_t *MQAutoQuad::EnumString(void)
{
	return L"Auto Quad";
}

//---------------------------------------------------------------------------
//  SingleMovePlugin::Initialize
//    アプリケーションの初期化
//---------------------------------------------------------------------------
BOOL MQAutoQuad::Initialize()
{
	return TRUE;
}

//---------------------------------------------------------------------------
//  SingleMovePlugin::Exit
//    アプリケーションの終了
//---------------------------------------------------------------------------
void MQAutoQuad::Exit()
{
}

//---------------------------------------------------------------------------
//  SingleMovePlugin::Activate
//    表示・非表示切り替え要求
//---------------------------------------------------------------------------
BOOL MQAutoQuad::Activate(MQDocument doc, BOOL flag)
{
	if (flag)
	{
		mqSnap.Update(doc);
		mqGeom.Clear();
	}
	else
	{
		clear(true, true, true);
	}

	m_bActivated = flag ? true : false;
	// It returns 'flag' as it is.
	// そのままflagを返す
	return flag;
}

void DrawFace(MQScene scene,MQObject draw , MQObject obj, const std::vector<int>& quad, int iMat0 )
{
	std::vector<int> pts; pts.reserve(quad.size());
	for (auto q : quad)
	{
		auto v = obj->GetVertex(q);
		pts.push_back(draw->AddVertex(v));
	}

	auto iFace = draw->AddFace((int)pts.size(), pts.data());
	draw->SetFaceMaterial(iFace, iMat0);

	std::reverse(pts.begin(), pts.end());
 	iFace = draw->AddFace((int)pts.size(), pts.data());
	draw->SetFaceMaterial(iFace, iMat0);
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::OnDraw
//    描画時の処理
//---------------------------------------------------------------------------
void MQAutoQuad::OnDraw(MQDocument doc, MQScene scene, int width, int height)
{
	if (!m_bActivated) return;

	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());
	MQColor color =  GetSystemColor(MQSYSTEMCOLOR_TEMP);

#if _DEBUG
	MQObject drawPoint = CreateDrawingObject(doc, DRAW_OBJECT_POINT);
	for( const auto& v : unk3  )
	{
		auto x = drawPoint->AddVertex(v);
		drawPoint->AddFace(1, &x);
	}

	MQObject drawEdge = CreateDrawingObject(doc, DRAW_OBJECT_LINE);
	for (const auto& edge : border.edges)
	{
		MQPoint p0 = edge->verts[0]->co;
		MQPoint p1 = edge->verts[1]->co;

		drawEdge->SetColor(MQColor(1, 1, 0));
		drawEdge->SetColorValid(TRUE);

		int newface[2];
		newface[0] = drawEdge->AddVertex(p0);
		newface[1] = drawEdge->AddVertex(p1);
		drawEdge->AddFace(2, newface);
	}
#endif

	if (!Quad.empty())
	{
		MQObject drawQuad = CreateDrawingObject(doc, DRAW_OBJECT_FACE);
		drawQuad->AddRenderFlag(MQOBJECT_RENDER_ALPHABLEND);
		drawQuad->SetColor(color);
		drawQuad->SetColorValid(TRUE);

		MQObject halfQuad = CreateDrawingObject(doc, DRAW_OBJECT_FACE);
		halfQuad->AddRenderFlag(MQOBJECT_RENDER_ALPHABLEND);
		halfQuad->AddRenderFlag(MQOBJECT_RENDER_OVERWRITEFACE);
		halfQuad->SetColor(color);
		halfQuad->SetColorValid(TRUE);

		int iMaterial0;
		auto mat0 = CreateDrawingMaterial(doc, iMaterial0);
		mat0->SetShader(MQMATERIAL_SHADER_CONSTANT);
		mat0->SetColor(color);
		mat0->SetAlpha(1.0f);

		int iMaterial1;
		auto mat1 = CreateDrawingMaterial(doc, iMaterial1);
		mat1->SetShader(MQMATERIAL_SHADER_CONSTANT);
		mat1->SetColor(color);
		mat1->SetAlpha(0.25f);

		DrawFace(scene, drawQuad, obj, Quad, iMaterial0);
		DrawFace(scene, halfQuad, obj, Quad, iMaterial1);

		if (Quad.size() == Mirror.size())
		{
			DrawFace(scene, drawQuad, obj, Mirror, iMaterial0);
			DrawFace(scene, halfQuad, obj, Mirror, iMaterial1);
		}
	}
}



//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonDown
//    左ボタンが押されたとき
//---------------------------------------------------------------------------
BOOL MQAutoQuad::OnLeftButtonDown(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	MQVector mouse_pos = MQPoint((float)state.MousePos.x, (float)state.MousePos.y, 0);
	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());
	if (obj->GetLocking() == TRUE || obj->GetVisible() == FALSE)
	{
		return FALSE;
	}

	if (Quad.size() >= 3)
	{
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
		mqGeom.Clear();
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
BOOL MQAutoQuad::OnLeftButtonMove(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	return FALSE;
}


//---------------------------------------------------------------------------
//  SingleMovePlugin::OnLeftButtonUp
//    左ボタンが離されたとき
//---------------------------------------------------------------------------
BOOL MQAutoQuad::OnLeftButtonUp(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	return FALSE;
}



//---------------------------------------------------------------------------
//  SingleMovePlugin::OnMouseMove
//    マウスが移動したとき
//---------------------------------------------------------------------------
BOOL MQAutoQuad::OnMouseMove(MQDocument doc, MQScene scene, MOUSE_BUTTON_STATE& state)
{
	bool redraw = false;
	auto mouse_pos = MQPoint((float)state.MousePos.x, (float)state.MousePos.y, 0);
	MQObject obj = doc->GetObject(doc->GetCurrentObjectIndex());

	if (obj->GetLocking() == TRUE || obj->GetVisible() == FALSE)
	{
		return FALSE;
	}

	mqGeom.Update(obj);
	border.Update(scene, mqGeom.obj);
	mqSnap.Update(doc);

#if _DEBUG
	unk3.clear();
	for (const MQPoint& v : mqGeom.obj->cos )
	{
		redraw = true;
		if (mqSnap.check_view(scene, v) == true)
		{
			unk3.push_back(v);
		}
	}
#endif

	std::vector<int> new_quad;
	std::vector<int> new_mirror;

	HIT_TEST_PARAM param;
	param.TestVertex = FALSE;
	param.TestLine = FALSE;
	param.TestFace = TRUE;
	std::vector<MQObject> objlist;
	objlist.push_back(obj);
	this->HitTestObjects(scene, state.MousePos, objlist, param);
	bool is_blank_area = true;
	if (param.HitType == MQCommandPlugin::HIT_TYPE::HIT_TYPE_FACE)
	{
		is_blank_area = !mqSnap.check_view(scene, param.HitPos);
	}

	if (is_blank_area)
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

	if (redraw) {
		RedrawScene(scene);
	}

	// It returns FALSE for a default action.
	// 独自処理を行ったが、標準動作も行わせるためFALSEを返す
	return FALSE;
}


std::pair< std::vector<int>, std::vector<int> > MQAutoQuad::FindQuad(MQDocument doc, MQScene scene, const MQPoint& mouse_pos)
{
	//スクリーン変換
	auto screen = sceneCache.Get(scene, mqGeom.obj );

	// 頂点の射影変換
	typedef std::pair< const MQGeom::Vert* , float > pair;
	auto vertset = std::vector< pair >();
	vertset.reserve( mqGeom.obj->verts.size() );
	for ( const auto vi : border.verts)
	{
		if (screen.in_screen[vi->id])
		{
			float dx = mouse_pos.x - screen.coords[vi->id].x;
			float dy = mouse_pos.y - screen.coords[vi->id].y;
			float dist = dx * dx + dy * dy;
			vertset.push_back( pair(vi, dist) );
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
		auto p = screen.coords[v->id];
		auto c = v->co;

		if (mqSnap.check_view(scene, c) == false)
		{
			continue;
		}

		bool isHitOtherEdge = false;
		for (const auto& border : border.edges)
		{
			auto a = border->verts[0];
			auto b = border->verts[1];
			if (IntersectLineAndLine(mouse_pos, p, screen.coords[a->id], screen.coords[b->id]))
			{
				if (a->id != v->id && b->id != v->id)
				{
					auto pos = IntersectLineAndLinePos(mouse_pos, p, screen.coords[a->id], screen.coords[b->id]);
					auto ray = MQRay(scene, pos );
					auto hit = ray.intersect( MQRay( a->co , b->co - a->co ) );
					if ( mqSnap.check_view(scene, hit.second ) )
					{
						isHitOtherEdge = true;
						break;
					}
				}
			}
		}
		if (!isHitOtherEdge)
		{
			new_quad.push_back( v->id );
			if (new_quad.size() >= 4)
			{
				auto tmp_quad = MakeQuad(new_quad, mqGeom.obj->cos, screen.coords , mouse_pos);
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
				const auto e0 = &mqGeom.obj->verts[ new_quad[i] ];
				const auto e1 = &mqGeom.obj->verts[ new_quad[(i + 1) % new_quad.size()] ];
				const MQGeom::Edge* edge = mqGeom.obj->find( e0 , e1 );
				if (std::find(border.edges.begin(), border.edges.end(), edge) == border.edges.end())
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
			mirror = FindMirror(mqGeom.obj->obj, mqGeom.obj->cos, new_quad, option.SymmetryDistance);
		}
	}

	return std::pair< std::vector<int>, std::vector<int> >(new_quad, mirror);
}


int MQAutoQuad::AddFace(MQScene scene, MQObject obj, std::vector<int> verts, int iMaterial)
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

std::vector<int> MQAutoQuad::FindMirror(MQObject obj, const std::vector<MQPoint>& verts, const std::vector<int>& poly, float SymmetryDistance)
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
	static MQAutoQuad plugin;
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

