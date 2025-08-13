<script setup>
import { RouterView, useRoute, useRouter } from 'vue-router'
import { computed, ref, watch } from 'vue'
import Sidebar from './components/parts/Sidebar.vue'
import Header from './components/parts/Header.vue'
import SettingsModal from './components/parts/SettingsModal.vue'
import NotificationModal from './components/parts/NotificationModal.vue'

const route = useRoute()
const router = useRouter()

// 현재 라우트가 로그인 페이지인지 확인
const isLoginPage = computed(() => route.name === 'Login')

// 로그인 페이지가 아닌 경우에만 레이아웃 컴포넌트 표시
const showLayout = computed(() => !isLoginPage.value)

// 모달 상태 관리
const showSettingsModal = ref(false)
const showNotificationModal = ref(false)

// 페이지 상태 관리
const currentPageTitle = ref('Administrator')
const notificationCount = ref(6)
const currentMenu = ref('dashboard')

// 메뉴 변경 핸들러
const handleMenuChange = (menu) => {
  currentMenu.value = menu
  updatePageTitle(menu)
}

// 페이지 제목 업데이트
const updatePageTitle = (menu) => {
  const titles = {
    'dashboard': 'Dashboard',
    'robot-status': 'Robot Status',
    'likes': 'Likes',
    'system-logs': 'System Logs',
    'robot-position': 'Robot Position'
  }
  currentPageTitle.value = titles[menu] || 'Administrator'
}

// 프로필 클릭 핸들러
const handleProfileClick = () => {
  console.log('프로필 클릭됨')
}

// Settings 모달 열기
const openSettingsModal = () => {
  showSettingsModal.value = true
  console.log('Settings 모달 열기:', showSettingsModal.value)
}

// Notification 모달 열기
const openNotificationModal = () => {
  showNotificationModal.value = true
  console.log('Notification 모달 열기:', showNotificationModal.value)
}

// 설정 저장 핸들러
const handleSettingsSaved = (settings) => {
  console.log('설정 저장됨:', settings)
  // 여기에 설정 저장 로직 추가
}

// 알림 개수 업데이트
const updateNotificationCount = (count) => {
  notificationCount.value = count
}

// 라우터 변경 감지하여 페이지 제목 업데이트
watch(() => route.name, (newRouteName) => {
  if (newRouteName && !isLoginPage.value) {
    const routeToMenuMap = {
      'Administrator': 'main',
      'DashboardDetail': 'dashboard',
      'RobotStatus': 'robot-status',
      'Log': 'system-logs',
      'Likes': 'likes',
      'RobotPosition': 'robot-position',
      'WebRTCTest': 'driving-stream'
    }
    
    const menu = routeToMenuMap[newRouteName]
    if (menu) {
      currentMenu.value = menu
      updatePageTitle(menu)
    }
  }
}, { immediate: true })
</script>

<template>
  <div>
    <!-- 로그인 페이지가 아닌 경우에만 레이아웃 표시 -->
    <template v-if="showLayout">
      <div class="dashboard">
        <!-- 사이드바 -->
        <Sidebar 
          @menu-change="handleMenuChange" 
          @open-settings="openSettingsModal"
        />

        <!-- 메인 콘텐츠 -->
        <div class="main-content">
          <!-- 헤더 -->
          <Header 
            :title="currentPageTitle"
            :notification-count="notificationCount"
            @notification-click="openNotificationModal"
            @profile-click="handleProfileClick"
            @notification-count-changed="updateNotificationCount"
          />

          <!-- 라우터 뷰 -->
          <RouterView />
        </div>

        <!-- 모달들 -->
        <SettingsModal 
          v-model="showSettingsModal"
          @settings-saved="handleSettingsSaved"
        />
        
        <NotificationModal 
          v-model="showNotificationModal"
          @notification-count-changed="updateNotificationCount"
        />
      </div>
    </template>

    <!-- 로그인 페이지인 경우 레이아웃 없이 라우터 뷰만 표시 -->
    <template v-else>
      <RouterView />
    </template>
  </div>
</template>

<style scoped>
.dashboard {
  display: flex;
  height: 100vh;
  background-color: #081028;
  color: #ffffff;
}

.main-content {
  flex: 1;
  padding: 20px;
  overflow-y: auto;
}

/* 반응형 디자인 */
@media (max-width: 768px) {
  .dashboard {
    flex-direction: column;
  }
  
  .main-content {
    padding: 10px;
  }
}
</style>
