<template>
  <div class="card chat-card">
    <div class="card-header">
      <button class="card-title-btn chat-btn">Chat</button>
    </div>
    <div class="card-content">
      <div v-if="loading" class="loading-container">
        <v-progress-circular indeterminate color="#3a57e8"></v-progress-circular>
        <span>채팅방 로딩 중...</span>
      </div>
      
      <div v-else-if="chatRooms.length === 0" class="empty-state">
        <v-icon size="48" color="#8a92a6">mdi-chat-outline</v-icon>
        <p>활성 채팅방이 없습니다</p>
      </div>
      
      <div v-else class="chat-grid">
        <div 
          class="chat-button" 
          v-for="room in chatRooms" 
          :key="room.id"
          @click="openChat(room)"
        >
          <div class="chat-info">
            <span class="order-number">{{ room.orderNumber }}</span>
            <span class="last-message">{{ room.lastMessage }}</span>
          </div>
          <div class="chat-meta">
            <span class="timestamp">{{ formatTime(room.lastUpdated) }}</span>
            <span v-if="room.unreadCount > 0" class="unread-badge">{{ room.unreadCount }}</span>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import { getChatRooms } from '@/api/examples.js'
// Props
const props = defineProps({
  // 필요한 경우 props 추가
})

// Emits
const emit = defineEmits(['open-chat'])

// Reactive data
const loading = ref(false)
const chatRooms = ref([])

// Methods
const fetchChatRooms = async () => {
  try {
    loading.value = true
    const response = await getChatRooms()
    chatRooms.value = response.data || []
    console.log('채팅방 데이터 조회 완료:', chatRooms.value)
  } catch (error) {
    console.error('채팅방 조회 실패:', error)
    // 에러 시 더미 데이터 사용
    chatRooms.value = [
      {
        id: 1,
        orderNumber: '주문번호 001',
        lastMessage: '배달 완료되었습니다.',
        lastUpdated: new Date(Date.now() - 5 * 60 * 1000), // 5분 전
        unreadCount: 2
      },
      {
        id: 2,
        orderNumber: '주문번호 002',
        lastMessage: '로봇이 출발했습니다.',
        lastUpdated: new Date(Date.now() - 15 * 60 * 1000), // 15분 전
        unreadCount: 0
      },
      {
        id: 3,
        orderNumber: '주문번호 003',
        lastMessage: '주문이 확인되었습니다.',
        lastUpdated: new Date(Date.now() - 30 * 60 * 1000), // 30분 전
        unreadCount: 1
      }
    ]
  } finally {
    loading.value = false
  }
}

const openChat = (room) => {
  emit('open-chat', room)
}

const formatTime = (timestamp) => {
  if (!timestamp) return ''
  
  const now = new Date()
  const time = new Date(timestamp)
  const diffMs = now - time
  const diffMins = Math.floor(diffMs / (1000 * 60))
  const diffHours = Math.floor(diffMs / (1000 * 60 * 60))
  const diffDays = Math.floor(diffMs / (1000 * 60 * 60 * 24))
  
  if (diffMins < 1) return '방금 전'
  if (diffMins < 60) return `${diffMins}분 전`
  if (diffHours < 24) return `${diffHours}시간 전`
  if (diffDays < 7) return `${diffDays}일 전`
  
  return time.toLocaleDateString('ko-KR')
}

// API 함수 (실제 구현 시 import)


// Lifecycle
onMounted(() => {
  fetchChatRooms()
})
</script>

<style scoped>
.chat-card {
  background-color: #222738;
  border-radius: 12px;
  padding: 24px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  min-height: 280px;
  display: flex;
  flex-direction: column;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
  flex-shrink: 0;
}

.card-title-btn {
  background: none;
  border: none;
  color: #d3d3d3;
  font-size: 18px;
  font-weight: bold;
  cursor: pointer;
  transition: color 0.3s ease;
}

.card-title-btn:hover {
  color: #3a57e8;
}

.card-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  min-height: 0;
}

.loading-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 100%;
  gap: 12px;
  color: #8a92a6;
}

.empty-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 100%;
  color: #8a92a6;
  text-align: center;
}

.empty-state p {
  margin-top: 12px;
  font-size: 14px;
}

.chat-grid {
  display: grid;
  grid-template-columns: 1fr;
  gap: 8px;
  max-height: 200px;
  overflow-y: auto;
}

.chat-button {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px;
  background-color: #1a1f2e;
  border-radius: 8px;
  cursor: pointer;
  transition: background-color 0.3s ease;
  border: 1px solid transparent;
}

.chat-button:hover {
  background-color: #2a2f3e;
  border-color: #3a57e8;
}

.chat-info {
  display: flex;
  flex-direction: column;
  gap: 4px;
  flex: 1;
  min-width: 0;
}

.order-number {
  font-weight: bold;
  color: #ffffff;
  font-size: 14px;
}

.last-message {
  color: #8a92a6;
  font-size: 12px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.chat-meta {
  display: flex;
  flex-direction: column;
  align-items: flex-end;
  gap: 4px;
}

.timestamp {
  color: #8a92a6;
  font-size: 11px;
}

.unread-badge {
  background-color: #ff4757;
  color: white;
  border-radius: 50%;
  width: 18px;
  height: 18px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 10px;
  font-weight: bold;
}

/* 스크롤바 스타일링 */
.chat-grid::-webkit-scrollbar {
  width: 6px;
}

.chat-grid::-webkit-scrollbar-track {
  background: #1a1f2e;
  border-radius: 3px;
}

.chat-grid::-webkit-scrollbar-thumb {
  background: #3a57e8;
  border-radius: 3px;
}

.chat-grid::-webkit-scrollbar-thumb:hover {
  background: #2a47d8;
}
</style>
