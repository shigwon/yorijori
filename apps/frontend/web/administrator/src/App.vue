<script setup>
import { RouterView, useRoute } from 'vue-router'
import { computed } from 'vue'
import Sidebar from './components/parts/Sidebar.vue'
import Header from './components/parts/Header.vue'

const route = useRoute()

// 현재 라우트가 로그인 페이지인지 확인
const isLoginPage = computed(() => route.name === 'Login')

// 로그인 페이지가 아닌 경우에만 레이아웃 컴포넌트 표시
const showLayout = computed(() => !isLoginPage.value)
</script>

<template>
  <div>
    <!-- 로그인 페이지가 아닌 경우에만 레이아웃 표시 -->
    <template v-if="showLayout">
      <div class="dashboard">
        <!-- 사이드바 -->
        <Sidebar @menu-change="handleMenuChange" />

        <!-- 메인 콘텐츠 -->
        <div class="main-content">
          <!-- 헤더 -->
          <Header 
            :title="currentPageTitle"
            :notification-count="notificationCount"
            @notification-click="handleNotificationClick"
            @profile-click="handleProfileClick"
          />

          <!-- 라우터 뷰 -->
          <RouterView />
        </div>
      </div>
    </template>

    <!-- 로그인 페이지인 경우 레이아웃 없이 라우터 뷰만 표시 -->
    <template v-else>
      <RouterView />
    </template>
  </div>
</template>

<script>
// Options API로 전역 상태 관리
export default {
  data() {
    return {
      currentPageTitle: 'Administrator',
      notificationCount: 6,
      currentMenu: 'dashboard'
    }
  },
  methods: {
    handleMenuChange(menu) {
      this.currentMenu = menu
      this.updatePageTitle(menu)
    },
    updatePageTitle(menu) {
      const titles = {
        'dashboard': 'Dashboard',
        'robot-status': 'Robot Status',
        'likes': 'Likes',
        'chat': 'Chat',
        'calendar': 'Calendar',
        'system-logs': 'System Logs',
        'driving-stream': 'Driving Stream',
        'robot-position': 'Robot Position'
      }
      this.currentPageTitle = titles[menu] || 'Administrator'
    },
    handleNotificationClick() {
      console.log('알림 클릭됨')
    },
    handleProfileClick() {
      console.log('프로필 클릭됨')
    }
  }
}
</script>

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
