<template>
  <v-dialog v-model="isOpen" max-width="500px" persistent>
    <v-card class="notification-modal" dark>
      <v-card-title class="notification-header">
        <v-icon class="notification-icon">mdi-bell</v-icon>
        <span class="notification-title">알림</span>
        <v-spacer></v-spacer>
        <v-btn icon @click="closeModal" class="close-btn">
          <v-icon>mdi-close</v-icon>
        </v-btn>
      </v-card-title>

      <v-card-text class="notification-content">
        <div class="notification-filters">
          <v-chip-group v-model="selectedFilter" mandatory>
            <v-chip value="all" color="#3dade5" variant="outlined">
              전체 ({{ notifications.length }})
            </v-chip>
            <v-chip value="robot" color="#3dade5" variant="outlined">
              로봇 ({{ robotNotifications.length }})
            </v-chip>
            <v-chip value="system" color="#3dade5" variant="outlined">
              시스템 ({{ systemNotifications.length }})
            </v-chip>
            <v-chip value="warning" color="#ff6b6b" variant="outlined">
              경고 ({{ warningNotifications.length }})
            </v-chip>
          </v-chip-group>
        </div>

        <div class="notification-list">
          <div v-if="filteredNotifications.length === 0" class="empty-state">
            <v-icon class="empty-icon">mdi-bell-off</v-icon>
            <p class="empty-text">새로운 알림이 없습니다</p>
          </div>

          <div
            v-for="notification in filteredNotifications"
            :key="notification.id"
            class="notification-item"
            :class="{
              'notification-unread': !notification.read,
              'notification-warning': notification.type === 'warning',
              'notification-error': notification.type === 'error'
            }"
            @click="markAsRead(notification.id)"
          >
            <div class="notification-icon-wrapper">
              <v-icon 
                :class="getNotificationIcon(notification.type).class"
                :color="getNotificationIcon(notification.type).color"
              >
                {{ getNotificationIcon(notification.type).icon }}
              </v-icon>
            </div>
            
            <div class="notification-content-wrapper">
              <div class="notification-header-content">
                <h4 class="notification-title-text">{{ notification.title }}</h4>
                <span class="notification-time">{{ formatTime(notification.timestamp) }}</span>
              </div>
              <p class="notification-message">{{ notification.message }}</p>
              <div v-if="notification.details" class="notification-details">
                <span class="details-label">상세정보:</span>
                <span class="details-text">{{ notification.details }}</span>
              </div>
            </div>

            <div class="notification-actions">
              <v-btn
                icon
                size="small"
                @click.stop="deleteNotification(notification.id)"
                class="delete-btn"
              >
                <v-icon>mdi-delete</v-icon>
              </v-btn>
            </div>
          </div>
        </div>
      </v-card-text>

      <v-card-actions class="notification-actions-footer">
        <v-spacer></v-spacer>
        <v-btn
          color="grey"
          variant="text"
          @click="markAllAsRead"
          :disabled="unreadCount === 0"
          class="mark-all-read-btn"
        >
          모두 읽음 처리
        </v-btn>
        <v-btn
          color="grey"
          variant="text"
          @click="clearAllNotifications"
          :disabled="notifications.length === 0"
          class="clear-all-btn"
        >
          모두 삭제
        </v-btn>
        <v-btn
          color="#3dade5"
          @click="closeModal"
          class="close-modal-btn"
        >
          닫기
        </v-btn>
      </v-card-actions>
    </v-card>
  </v-dialog>
</template>

<script setup>
import { ref, computed, watch } from 'vue'

const props = defineProps({
  modelValue: {
    type: Boolean,
    default: false
  }
})

const emit = defineEmits(['update:modelValue', 'notification-count-changed'])

const isOpen = ref(false)
const selectedFilter = ref('all')

// 알림 데이터 (실제로는 API에서 가져올 데이터)
const notifications = ref([
  {
    id: 1,
    type: 'robot',
    title: '로봇 상태 변경',
    message: 'Robot-001이 대기 상태에서 운행 상태로 변경되었습니다.',
    details: 'Robot ID: R001, 위치: 3층 로비',
    timestamp: new Date(Date.now() - 5 * 60 * 1000), // 5분 전
    read: false
  },
  {
    id: 2,
    type: 'warning',
    title: '배터리 부족 경고',
    message: 'Robot-002의 배터리 잔량이 20% 이하로 떨어졌습니다.',
    details: 'Robot ID: R002, 배터리: 18%',
    timestamp: new Date(Date.now() - 15 * 60 * 1000), // 15분 전
    read: false
  },
  {
    id: 3,
    type: 'system',
    title: '시스템 업데이트 완료',
    message: '로봇 관리 시스템이 성공적으로 업데이트되었습니다.',
    details: '버전: v2.1.0, 업데이트 시간: 2024-01-15 14:30',
    timestamp: new Date(Date.now() - 2 * 60 * 60 * 1000), // 2시간 전
    read: true
  },
  {
    id: 4,
    type: 'robot',
    title: '새로운 로봇 등록',
    message: '새로운 로봇 Robot-003이 시스템에 등록되었습니다.',
    details: 'Robot ID: R003, 모델: Linky Pro',
    timestamp: new Date(Date.now() - 4 * 60 * 60 * 1000), // 4시간 전
    read: true
  },
  {
    id: 5,
    type: 'warning',
    title: '네트워크 연결 불안정',
    message: 'Robot-001과의 네트워크 연결이 불안정합니다.',
    details: 'Robot ID: R001, 신호 강도: 약함',
    timestamp: new Date(Date.now() - 6 * 60 * 60 * 1000), // 6시간 전
    read: false
  },
  {
    id: 6,
    type: 'system',
    title: '데이터베이스 백업 완료',
    message: '일일 데이터베이스 백업이 성공적으로 완료되었습니다.',
    details: '백업 크기: 2.3GB, 소요 시간: 15분',
    timestamp: new Date(Date.now() - 24 * 60 * 60 * 1000), // 1일 전
    read: true
  }
])

// 모달 열기/닫기
watch(() => props.modelValue, (newVal) => {
  isOpen.value = newVal
})

watch(isOpen, (newVal) => {
  emit('update:modelValue', newVal)
})

// 필터링된 알림
const filteredNotifications = computed(() => {
  if (selectedFilter.value === 'all') {
    return notifications.value
  }
  return notifications.value.filter(notification => notification.type === selectedFilter.value)
})

// 카테고리별 알림 개수
const robotNotifications = computed(() => 
  notifications.value.filter(n => n.type === 'robot')
)

const systemNotifications = computed(() => 
  notifications.value.filter(n => n.type === 'system')
)

const warningNotifications = computed(() => 
  notifications.value.filter(n => n.type === 'warning')
)

// 읽지 않은 알림 개수
const unreadCount = computed(() => 
  notifications.value.filter(n => !n.read).length
)

// 알림 아이콘 설정
const getNotificationIcon = (type) => {
  const icons = {
    robot: { icon: 'mdi-robot', color: '#3dade5', class: 'robot-icon' },
    system: { icon: 'mdi-cog', color: '#8a92a6', class: 'system-icon' },
    warning: { icon: 'mdi-alert', color: '#ff6b6b', class: 'warning-icon' },
    error: { icon: 'mdi-alert-circle', color: '#ff4757', class: 'error-icon' }
  }
  return icons[type] || icons.system
}

// 시간 포맷팅
const formatTime = (timestamp) => {
  const now = new Date()
  const diff = now - timestamp
  const minutes = Math.floor(diff / (1000 * 60))
  const hours = Math.floor(diff / (1000 * 60 * 60))
  const days = Math.floor(diff / (1000 * 60 * 60 * 24))

  if (minutes < 60) {
    return `${minutes}분 전`
  } else if (hours < 24) {
    return `${hours}시간 전`
  } else {
    return `${days}일 전`
  }
}

// 알림 읽음 처리
const markAsRead = (id) => {
  const notification = notifications.value.find(n => n.id === id)
  if (notification && !notification.read) {
    notification.read = true
    updateNotificationCount()
  }
}

// 모든 알림 읽음 처리
const markAllAsRead = () => {
  notifications.value.forEach(notification => {
    notification.read = true
  })
  updateNotificationCount()
}

// 알림 삭제
const deleteNotification = (id) => {
  const index = notifications.value.findIndex(n => n.id === id)
  if (index > -1) {
    notifications.value.splice(index, 1)
    updateNotificationCount()
  }
}

// 모든 알림 삭제
const clearAllNotifications = () => {
  notifications.value = []
  updateNotificationCount()
}

// 알림 개수 업데이트
const updateNotificationCount = () => {
  emit('notification-count-changed', unreadCount.value)
}

const closeModal = () => {
  isOpen.value = false
}
</script>

<style scoped>
.notification-modal {
  background-color: #1a1f2e !important;
  color: #ffffff !important;
}

.notification-header {
  background-color: #2a2f3e;
  border-bottom: 1px solid #3a3f4e;
  padding: 20px 24px;
}

.notification-icon {
  color: #3dade5;
  margin-right: 12px;
  font-size: 24px;
}

.notification-title {
  font-size: 20px;
  font-weight: 600;
  color: #ffffff;
}

.close-btn {
  color: #8a92a6;
}

.close-btn:hover {
  color: #ffffff;
}

.notification-content {
  padding: 0;
  max-height: 500px;
  overflow-y: auto;
}

.notification-filters {
  padding: 20px 24px;
  border-bottom: 1px solid #3a3f4e;
  background-color: #2a2f3e;
}

.notification-list {
  padding: 0;
}

.empty-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 60px 20px;
  color: #8a92a6;
}

.empty-icon {
  font-size: 48px;
  margin-bottom: 16px;
  opacity: 0.5;
}

.empty-text {
  font-size: 16px;
  margin: 0;
}

.notification-item {
  display: flex;
  align-items: flex-start;
  padding: 16px 24px;
  border-bottom: 1px solid #3a3f4e;
  cursor: pointer;
  transition: background-color 0.2s ease;
}

.notification-item:hover {
  background-color: #2a2f3e;
}

.notification-item:last-child {
  border-bottom: none;
}

.notification-unread {
  background-color: rgba(61, 173, 229, 0.1);
  border-left: 3px solid #3dade5;
}

.notification-warning {
  border-left: 3px solid #ff6b6b;
}

.notification-error {
  border-left: 3px solid #ff4757;
}

.notification-icon-wrapper {
  margin-right: 16px;
  margin-top: 2px;
}

.notification-content-wrapper {
  flex: 1;
  min-width: 0;
}

.notification-header-content {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  margin-bottom: 8px;
}

.notification-title-text {
  font-size: 14px;
  font-weight: 600;
  color: #ffffff;
  margin: 0;
  line-height: 1.4;
}

.notification-time {
  font-size: 12px;
  color: #8a92a6;
  white-space: nowrap;
  margin-left: 12px;
}

.notification-message {
  font-size: 13px;
  color: #c8d1e0;
  margin: 0 0 8px 0;
  line-height: 1.4;
}

.notification-details {
  font-size: 12px;
  color: #8a92a6;
}

.details-label {
  font-weight: 500;
  margin-right: 8px;
}

.details-text {
  color: #c8d1e0;
}

.notification-actions {
  margin-left: 12px;
}

.delete-btn {
  color: #8a92a6;
}

.delete-btn:hover {
  color: #ff6b6b;
}

.notification-actions-footer {
  background-color: #2a2f3e;
  border-top: 1px solid #3a3f4e;
  padding: 16px 24px;
}

.mark-all-read-btn {
  color: #3dade5;
}

.clear-all-btn {
  color: #ff6b6b;
}

.close-modal-btn {
  background-color: #3dade5;
  color: #ffffff;
  font-weight: 500;
}

.close-modal-btn:hover {
  background-color: #2b8bc7;
}

/* Vuetify 컴포넌트 스타일 오버라이드 */
:deep(.v-chip) {
  background-color: transparent;
  border-color: #3a3f4e;
  color: #8a92a6;
}

:deep(.v-chip--selected) {
  background-color: #3dade5;
  border-color: #3dade5;
  color: #ffffff;
}

:deep(.v-chip-group) {
  gap: 8px;
}

/* 스크롤바 스타일 */
.notification-content::-webkit-scrollbar {
  width: 6px;
}

.notification-content::-webkit-scrollbar-track {
  background: #2a2f3e;
}

.notification-content::-webkit-scrollbar-thumb {
  background: #3a3f4e;
  border-radius: 3px;
}

.notification-content::-webkit-scrollbar-thumb:hover {
  background: #4a4f5e;
}
</style>
