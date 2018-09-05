/****************************************************************************
 *
 *   Copyright (c) 2013,2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file geofence.cpp
 * Provides functions for handling the geofence
 *
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */
#include "geofence.h"
#include "navigator.h"

#include <ctype.h>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <systemlib/mavlink_log.h>

#include "navigator.h"

#define GEOFENCE_RANGE_WARNING_LIMIT 5000000

Geofence::Geofence(Navigator *navigator) :
	ModuleParams(navigator),
	_navigator(navigator),
	_sub_airdata(ORB_ID(vehicle_air_data))
{
	// we assume there's no concurrent fence update on startup
	_updateFence();
}

Geofence::~Geofence()
{
	if (_polygons) {
		delete[](_polygons);
	}
}

//실제 geofence 를 생성하는데 실제 동작은 _updateFence()에서 구현
void Geofence::updateFence()
{
	// Note: be aware that when calling this, it can block for quite some time, the duration of a geofence transfer.
	// However this is currently not used
	int ret = dm_lock(DM_KEY_FENCE_POINTS);

	if (ret != 0) {
		PX4_ERR("lock failed");
		return;
	}

	_updateFence();
	dm_unlock(DM_KEY_FENCE_POINTS);
}

// dm에서 fence를 정보를 읽어서 _polygons 에 각 다각형의 정보를 저장. _polygons에는 첫번째 다각형의 좌표정보, vertex의 수를 저장해 둔다.
// 추후 각 다각형의 첫번째 vertex 정보로 해당 다각형의 정보를 구성할 수 있다.
void Geofence::_updateFence()
{

	// fence 정보 가지고 오기 (geofence item 갯수를 알아야 loop 횟수를 알 수 있음)
	// initialize fence points count
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_fence_items = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_fence_items = stats.num_items;  //item의 수 (이 값만큼 loop돌면서 fence point 정보를 가지고와서 polygon 만들기 )
		_update_counter = stats.update_counter; //update 횟수
	}

	// iterate over all polygons and store their starting vertices
	_num_polygons = 0;
	int current_seq = 1;

	// 각 geofence의 시작 vertex을 저장.
	while (current_seq <= num_fence_items) {
		mission_fence_point_s mission_fence_point;
		bool is_circle_area = false;

		if (dm_read(DM_KEY_FENCE_POINTS, current_seq, &mission_fence_point, sizeof(mission_fence_point_s)) !=
		    sizeof(mission_fence_point_s)) {
			PX4_ERR("dm_read failed");
			break;
		}

		switch (mission_fence_point.nav_cmd) {
		case NAV_CMD_FENCE_RETURN_POINT:
			// TODO: do we need to store this?
			++current_seq;
			break;

		case NAV_CMD_FENCE_CIRCLE_INCLUSION:
		case NAV_CMD_FENCE_CIRCLE_EXCLUSION:
			is_circle_area = true;

		/* FALLTHROUGH */
		case NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION:
		case NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION:
			if (!is_circle_area && mission_fence_point.vertex_count == 0) {
				++current_seq; // avoid endless loop
				PX4_ERR("Polygon with 0 vertices. Skipping");

			} else {
				if (_polygons) {
					// resize: this is somewhat inefficient, but we do not expect there to be many polygons
					PolygonInfo *new_polygons = new PolygonInfo[_num_polygons + 1];

					if (new_polygons) {
						memcpy(new_polygons, _polygons, sizeof(PolygonInfo) * _num_polygons);
					}

					delete[](_polygons);
					_polygons = new_polygons;

				} else {
					_polygons = new PolygonInfo[1];
				}

				if (!_polygons) {
					_num_polygons = 0;
					PX4_ERR("alloc failed");
					return;
				}

				PolygonInfo &polygon = _polygons[_num_polygons];
				polygon.dataman_index = current_seq;
				polygon.fence_type = mission_fence_point.nav_cmd;

				if (is_circle_area) { // 원인 경우 원의 반경 값을 추가
					polygon.circle_radius = mission_fence_point.circle_radius;
					current_seq += 1;

				} else { //다각형인 경우 vertex가 몇 개 인지 추가
					polygon.vertex_count = mission_fence_point.vertex_count;
					current_seq += mission_fence_point.vertex_count; //current_seq 값을 vertex의 수만큼 건너뛰게 해서 다음 다각형의 첫번째 vertex를 가져온다.
				}

				++_num_polygons;
			}

			break;

		default:
			PX4_ERR("unhandled Fence command: %i", (int)mission_fence_point.nav_cmd);
			++current_seq;
			break;
		}

	}

}

// gpos가 기준 geofence를 지키는지 검사
bool Geofence::checkAll(const struct vehicle_global_position_s &global_position)
{
	return checkAll(global_position.lat, global_position.lon, global_position.alt);
}

// gpos와 지정한 고도가 geofence를 지키는지 검사
bool Geofence::checkAll(const struct vehicle_global_position_s &global_position, const float alt)
{
	return checkAll(global_position.lat, global_position.lon, alt);
}

// 설정한 고도 모드나 소드에 따라서 gpos나 gps_pos 중에 하나를 선택함
bool Geofence::check(const vehicle_global_position_s &global_position, const vehicle_gps_position_s &gps_position,
		     const home_position_s home_pos, bool home_position_set)
{
	if (getAltitudeMode() == Geofence::GF_ALT_MODE_WGS84) {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return checkAll(global_position);

		} else {
			return checkAll((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
					(double)gps_position.alt * 1.0e-3);
		}

	} else { //GF_ALT_MODE_WGS84 고도를 사용하지 않는 경우, baro 고도를 사용
		// get baro altitude
		_sub_airdata.update();
		const float baro_altitude_amsl = _sub_airdata.get().baro_alt_meter;

		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return checkAll(global_position, baro_altitude_amsl);

		} else {
			return checkAll((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7, baro_altitude_amsl);
		}
	}
}

//mission item기준으로 geofence 벗어나는지 검사
bool Geofence::check(const struct mission_item_s &mission_item)
{
	return checkAll(mission_item.lat, mission_item.lon, mission_item.altitude);
}

//lat, lon, 고도를 인자로 주고 geofence 벗어나는지 조사
bool Geofence::checkAll(double lat, double lon, float altitude)
{
	bool inside_fence = true;

	// home pos가 제공되는 경우
	if (isHomeRequired() && _navigator->home_position_valid()) {

		// home으로부터의 허용하는 최대 수평, 수직 거리
		const float max_horizontal_distance = _param_max_hor_distance.get();
		const float max_vertical_distance = _param_max_ver_distance.get();

		const double home_lat = _navigator->get_home_position()->lat;
		const double home_lon = _navigator->get_home_position()->lon;
		const double home_alt = _navigator->get_home_position()->alt;

		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		// home pos에서 현재 pos까지의 수평, 수직 거리 구하기 
		get_distance_to_point_global_wgs84(lat, lon, altitude, home_lat, home_lon, home_alt, &dist_xy, &dist_z);

		// 허용 수직거리보다 더 멀리 떨어진 경우. 고도가 벗어났다고 mavlink 메시지로 알림.
		if (max_vertical_distance > FLT_EPSILON && (dist_z > max_vertical_distance)) {
			if (hrt_elapsed_time(&_last_vertical_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Maximum altitude above home exceeded by %.1f m",
						     (double)(dist_z - max_vertical_distance));
				_last_vertical_range_warning = hrt_absolute_time();
			}

			inside_fence = false;
		}

		// 허용 수평거리보다 더 멀리 떨어진 경우. 거리가 벗어난 경우 mavlink 메시지로 알림.
		if (max_horizontal_distance > FLT_EPSILON && (dist_xy > max_horizontal_distance)) {
			if (hrt_elapsed_time(&_last_horizontal_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Maximum distance from home exceeded by %.1f m",
						     (double)(dist_xy - max_horizontal_distance));
				_last_horizontal_range_warning = hrt_absolute_time();
			}

			inside_fence = false;
		}
	}

	// 기본적으로 home과 허용하는 거리 내에 있으면서 polygon 내ㅇ에도 있어야 true가 됨.
	// to be inside the geofence both fences have to report being inside
	// as they both report being inside when not enabled
	inside_fence = inside_fence && checkPolygons(lat, lon, altitude);

	if (inside_fence) {
		_outside_counter = 0;
		return inside_fence;

	} else {
		_outside_counter++;

		// 여러 polygon 에서 몇 번 fence를 벗어난 횟수가 허용 범위 내에 있는지 체크
		if (_outside_counter > _param_counter_threshold.get()) {
			return inside_fence;

		} else {
			return true;
		}
	}
}

//lat, lon, 고도를 인자로 주고 이 지점이 벗어나는지 여부 조사
bool Geofence::checkPolygons(double lat, double lon, float altitude)
{
	// the following uses dm_read, so first we try to lock all items. If that fails, it (most likely) means
	// the data is currently being updated (via a mavlink geofence transfer), and we do not check for a violation now
	if (dm_trylock(DM_KEY_FENCE_POINTS) != 0) {
		return true;
	}

	// fence 정보가 업데이트 되었는지 검사하여 최신 fence정보로 업데이트. dm의 update_counter값과 변수값 비교해서 다르면 다시 fence 정보를 업데이트 한다.
	// we got the lock, now check if the fence data got updated
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (ret == sizeof(mission_stats_entry_s) && _update_counter != stats.update_counter) {
		_updateFence();
	}

	if (isEmpty()) {
		dm_unlock(DM_KEY_FENCE_POINTS);
		/* Empty fence -> accept all points */
		return true;
	}

	// 수직 거리 검사 : 인자로 받은 고도가 범위를 벗어나면 false 반환
	/* Vertical check */
	if (_altitude_max > _altitude_min) { // only enable vertical check if configured properly
		if (altitude > _altitude_max || altitude < _altitude_min) {
			dm_unlock(DM_KEY_FENCE_POINTS);
			return false;
		}
	}


	/* Horizontal check: iterate all polygons & circles */
	bool outside_exclusion = true;
	bool inside_inclusion = false;
	bool had_inclusion_areas = false;

	// geofence 개수만큼 반복. insideCircle(), insidePolygon() 사용해서 내부 여부 확인하고 이 결과를 반환
	for (int polygon_idx = 0; polygon_idx < _num_polygons; ++polygon_idx) {
		if (_polygons[polygon_idx].fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION) {
			bool inside = insideCircle(_polygons[polygon_idx], lat, lon, altitude);

			if (inside) {
				inside_inclusion = true;
			}

			had_inclusion_areas = true;

		} else if (_polygons[polygon_idx].fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			bool inside = insideCircle(_polygons[polygon_idx], lat, lon, altitude);

			if (inside) {
				outside_exclusion = false;
			}

		} else { // it's a polygon
			bool inside = insidePolygon(_polygons[polygon_idx], lat, lon, altitude);

			if (_polygons[polygon_idx].fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION) {
				if (inside) {
					inside_inclusion = true;
				}

				had_inclusion_areas = true;

			} else { // exclusion
				if (inside) {
					outside_exclusion = false;
				}
			}
		}
	}

	dm_unlock(DM_KEY_FENCE_POINTS);

	return (!had_inclusion_areas || inside_inclusion) && outside_exclusion;
}

// polygon 구조체로 geofence의 각 vertex(lat, lon, 고도 등) 정보를 읽어와서 인자로 받은 lat, lon, 고도가 fence 내부인지 여부 확인
bool Geofence::insidePolygon(const PolygonInfo &polygon, double lat, double lon, float altitude)
{
	//PNPOLY 알고리즘 이용해서 각 vertex의 좌표를 가지고 인자로 받은 lat, lon, 고도가 fence내에 포함되는지 여부를 확인

	/* Adaptation of algorithm originally presented as
	 * PNPOLY - Point Inclusion in Polygon Test
	 * W. Randolph Franklin (WRF)
	 * Only supports non-complex polygons (not self intersecting)
	 */

	mission_fence_point_s temp_vertex_i;
	mission_fence_point_s temp_vertex_j;
	bool c = false;

	// vertex의 처음 인덱스부터 증가, vertex의 마지막 인덱스부터 감소 시키는 방식으로 2개 vertex를 읽어서 처리
	for (unsigned i = 0, j = polygon.vertex_count - 1; i < polygon.vertex_count; j = i++) {
		//dm에서 앞쪽 vertex 읽기
		if (dm_read(DM_KEY_FENCE_POINTS, polygon.dataman_index + i, &temp_vertex_i,
			    sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s)) {
			break;
		}

		//dm에서 뒤쪽 vertex 읽기
		if (dm_read(DM_KEY_FENCE_POINTS, polygon.dataman_index + j, &temp_vertex_j,
			    sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s)) {
			break;
		}

		if (temp_vertex_i.frame != NAV_FRAME_GLOBAL && temp_vertex_i.frame != NAV_FRAME_GLOBAL_INT
		    && temp_vertex_i.frame != NAV_FRAME_GLOBAL_RELATIVE_ALT
		    && temp_vertex_i.frame != NAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
			// TODO: handle different frames
			PX4_ERR("Frame type %i not supported", (int)temp_vertex_i.frame);
			break;
		}

		//lon이 i와 j의 vertex 사이에 있어야 하고, ....
		if (((double)temp_vertex_i.lon >= lon) != ((double)temp_vertex_j.lon >= lon) &&
		    (lat <= (double)(temp_vertex_j.lat - temp_vertex_i.lat) * (lon - (double)temp_vertex_i.lon) /
		     (double)(temp_vertex_j.lon - temp_vertex_i.lon) + (double)temp_vertex_i.lat)) {
			c = !c;
		}
	}

	return c;
}

// 원 내부에 있는지 검사
bool Geofence::insideCircle(const PolygonInfo &polygon, double lat, double lon, float altitude)
{

	mission_fence_point_s circle_point;

	if (dm_read(DM_KEY_FENCE_POINTS, polygon.dataman_index, &circle_point,
		    sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s)) {
		PX4_ERR("dm_read failed");
		return false;
	}

	if (circle_point.frame != NAV_FRAME_GLOBAL && circle_point.frame != NAV_FRAME_GLOBAL_INT
	    && circle_point.frame != NAV_FRAME_GLOBAL_RELATIVE_ALT
	    && circle_point.frame != NAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
		// TODO: handle different frames
		PX4_ERR("Frame type %i not supported", (int)circle_point.frame);
		return false;
	}

	// 인자로 받은 lat, lon으로 ref 만들고
	if (!map_projection_initialized(&_projection_reference)) {
		map_projection_init(&_projection_reference, lat, lon);
	}

	float x1, y1, x2, y2;
	map_projection_project(&_projection_reference, lat, lon, &x1, &y1); //lat, lon 기준이므로 거
	map_projection_project(&_projection_reference, circle_point.lat, circle_point.lon, &x2, &y2); // 원의 특정 지점의 lat, lon 까지의 x, y 좌표
	float dx = x1 - x2, dy = y1 - y2; //현재 lat, lon 기준으로 원테두리 까지의 x, y 차이 구하기  
	//원점 기준으로의 거리가 반경 이내 인지 체크 (루트연산을 하지 않기 위해서 그냥 반경을 곱했음. )
	return dx * dx + dy * dy < circle_point.circle_radius * circle_point.circle_radius;
}

bool
Geofence::valid()
{
	return true; // always valid
}

// sd카드 /etc/geofence.txt 에 파일 넣어두면 이 파일을 읽어서 dm에 넣어줌
int
Geofence::loadFromFile(const char *filename)
{
	FILE		*fp;
	char		line[120];
	int			pointCounter = 0;
	bool		gotVertical = false;
	const char commentChar = '#';
	int rc = PX4_ERROR;

	// dm 삭제
	/* Make sure no data is left in the datamanager */
	clearDm();

	/* open the mixer definition file */
	fp = fopen(GEOFENCE_FILENAME, "r"); // /etc/geofence.txt 위치

	if (fp == nullptr) {
		return PX4_ERROR;
	}

	/* create geofence points from valid lines and store in DM */
	for (;;) {
		/* get a line, bail on error/EOF */
		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* Trim leading whitespace */
		size_t textStart = 0;

		while ((textStart < sizeof(line) / sizeof(char)) && isspace(line[textStart])) { textStart++; }

		/* if the line starts with #, skip */
		if (line[textStart] == commentChar) {
			continue;
		}

		/* if there is only a linefeed, skip it */
		if (line[0] == '\n') {
			continue;
		}

		if (gotVertical) {
			// 라인을 읽으면 그 값으로 geofence의 지점
			/* Parse the line as a geofence point */
			mission_fence_point_s vertex;
			vertex.frame = NAV_FRAME_GLOBAL;
			vertex.nav_cmd = NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION;
			vertex.vertex_count = 0; // this will be filled in a second pass
			vertex.alt = 0; // alt is not used

			// 라인 시작이 'DMS'로 시작. 도, 분, 초로 구성되었다는 뜻이므로 6개 읽기 
			// 파일에서 읽은 값을 vertex 변수에 넣으면 dm_write에서 vertex 변수 내용을 dm에 쓰게 됨.
			/* if the line starts with DMS, this means that the coordinate is given as degree minute second instead of decimal degrees */
			if (line[textStart] == 'D' && line[textStart + 1] == 'M' && line[textStart + 2] == 'S') {
				/* Handle degree minute second format */
				double lat_d, lat_m, lat_s, lon_d, lon_m, lon_s;

				if (sscanf(line, "DMS %lf %lf %lf %lf %lf %lf", &lat_d, &lat_m, &lat_s, &lon_d, &lon_m, &lon_s) != 6) {
					PX4_ERR("Scanf to parse DMS geofence vertex failed.");
					goto error;
				}

				// PX4_INFO("Geofence DMS: %.5lf %.5lf %.5lf ; %.5lf %.5lf %.5lf", lat_d, lat_m, lat_s, lon_d, lon_m, lon_s);

				vertex.lat = lat_d + lat_m / 60.0 + lat_s / 3600.0;
				vertex.lon = lon_d + lon_m / 60.0 + lon_s / 3600.0;

			} else { // DMS로 시작하지 않는 경우 lat, lon으로 2개만 읽음
				/* Handle decimal degree format */
				if (sscanf(line, "%lf %lf", &vertex.lat, &vertex.lon) != 2) {
					PX4_ERR("Scanf to parse geofence vertex failed.");
					goto error;
				}
			}
			// vertex 변수의 값을 dm에 저장 
			if (dm_write(DM_KEY_FENCE_POINTS, pointCounter + 1, DM_PERSIST_POWER_ON_RESET, &vertex,
				     sizeof(vertex)) != sizeof(vertex)) {
				goto error;
			}

			PX4_INFO("Geofence: point: %d, lat %.5lf: lon: %.5lf", pointCounter, vertex.lat, vertex.lon);

			pointCounter++; //파일에서 읽은 지점의 갯수 카운트

		} else { //goVertical은 무조건 여기 들어온다. 즉 파일의 시작에 바로 min/max를 먼저 읽어서 설정.
			/* Parse the line as the vertical limits */
			if (sscanf(line, "%f %f", &_altitude_min, &_altitude_max) != 2) {
				goto error;
			}

			PX4_INFO("Geofence: alt min: %.4f, alt_max: %.4f", (double)_altitude_min, (double)_altitude_max);
			gotVertical = true;
		}
	}

	// 정상적으로 sd카드에서 dm으로 쓰기 성공
	/* Check if import was successful */
	if (gotVertical && pointCounter > 2) {
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Geofence imported");
		rc = PX4_OK;

		// 정상적으로 dm에 들어갔으므로 이번에는 vertex_count를 증가시켜주고 나서 다시 저장
		/* do a second pass, now that we know the number of vertices */
		for (int seq = 1; seq <= pointCounter; ++seq) {
			mission_fence_point_s mission_fence_point;

			if (dm_read(DM_KEY_FENCE_POINTS, seq, &mission_fence_point, sizeof(mission_fence_point_s)) ==
			    sizeof(mission_fence_point_s)) {
				mission_fence_point.vertex_count = pointCounter; //vertex_count 증가
				dm_write(DM_KEY_FENCE_POINTS, seq, DM_PERSIST_POWER_ON_RESET, &mission_fence_point,
					 sizeof(mission_fence_point_s));
			}
		}

		// 최종적으로 polygon 갯수 정보 가지고 있는 stat도 DM_PERSIST_POWER_ON_RESET 속성 추가하기
		mission_stats_entry_s stats;
		stats.num_items = pointCounter;
		rc = dm_write(DM_KEY_FENCE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));

	} else {
		PX4_ERR("Geofence: import error");
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence import error");
	}

	updateFence();

error:
	fclose(fp);
	return rc;
}

//fence 정보 삭제
int Geofence::clearDm()
{
	dm_clear(DM_KEY_FENCE_POINTS);
	updateFence();
	return PX4_OK;
}

// home기주느 최대 수평, 수직 거리가 설정되어 있으므로 home위치를 알아야 가능. 그래서 home이 필요함
bool Geofence::isHomeRequired()
{
	bool max_horizontal_enabled = (_param_max_hor_distance.get() > FLT_EPSILON);
	bool max_vertical_enabled = (_param_max_ver_distance.get() > FLT_EPSILON);
	bool geofence_action_rtl = (getGeofenceAction() == geofence_result_s::GF_ACTION_RTL);

	return max_horizontal_enabled || max_vertical_enabled || geofence_action_rtl;
}

// 상태 출력 - polygon의 수와 vertex의 수를 출력
void Geofence::printStatus()
{
	int num_inclusion_polygons = 0, num_exclusion_polygons = 0, total_num_vertices = 0;
	int num_inclusion_circles = 0, num_exclusion_circles = 0;

	for (int i = 0; i < _num_polygons; ++i) {
		total_num_vertices += _polygons[i].vertex_count; // vertex 수를 계속 더함

		if (_polygons[i].fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION) {
			++num_inclusion_polygons; // 다각형 내부인 경우 증가
		}

		if (_polygons[i].fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION) {
			++num_exclusion_polygons; // 다각형 외부인 경우 증가
		}

		if (_polygons[i].fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION) {
			++num_inclusion_circles; // 원 내부인 경우 증가
		}

		if (_polygons[i].fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			++num_exclusion_circles; // 원 외부인 경우 증가
		}
	}

	PX4_INFO("Geofence: %i inclusion, %i exclusion polygons, %i inclusion, %i exclusion circles, %i total vertices",
		 num_inclusion_polygons, num_exclusion_polygons, num_inclusion_circles, num_exclusion_circles,
		 total_num_vertices);
}
