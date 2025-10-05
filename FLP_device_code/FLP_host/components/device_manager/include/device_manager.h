/*
 * device_manager.h
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include "common/common_types.h"
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief 장치 관리자를 초기화.
 * NVS에서 저장된 장치 목록을 불러와 메모리에 올립니다.
 */
void device_manager_init(void);

/**
 * @brief 새로운 장치를 목록에 추가.
 *
 * @param device_id 추가할 장치의 ID
 * @return true 추가 성공
 * @return false 목록이 꽉 찼거나 이미 존재하는 ID일 경우
 */
bool device_manager_add_device(int device_id);

/**
 * @brief 현재 등록된 장치의 수를 반환.
 *
 * @return size_t 등록된 장치의 수
 */
size_t device_manager_get_device_count(void);

/**
 * @brief 메모리에 있는 장치 정보를 인덱스로 조회.
 *
 * @param index 조회할 장치의 인덱스 (0부터 시작)
 * @param out_device 조회된 장치 정보를 복사할 구조체 포인터
 * @return true 조회 성공
 * @return false 유효하지 않은 인덱스일 경우
 */
bool device_manager_get_device_by_index(size_t index, TerminalDevice *out_device);

/**
 * @brief 장치의 정보를 ID로 찾아 업데이트.
 *
 * @param device_id 업데이트할 장치의 ID
 * @param lat 새로운 위도
 * @param lon 새로운 경도
 * @param danger 새로운 위험 정보
 * @return true 업데이트 성공
 * @return false 해당 ID를 가진 장치가 없을 경우
 */
bool device_manager_update_device_info(int device_id, double lat, double lon, uint8_t danger);

/**
 * @brief 장치 ID로 장치 정보를 조회.
 *
 * @param device_id 조회할 장치의 ID
 * @param out_device 조회된 장치 정보를 복사할 구조체 포인터
 * @return true 조회 성공
 * @return false 해당 ID를 가진 장치가 없을 경우
 */
bool device_manager_get_device_by_id(int device_id, TerminalDevice *out_device);

#endif // DEVICE_MANAGER_H
