<script setup>
// 고객 앱 상태 관리
import { ref, computed } from 'vue'
import WelcomeScreen from './components/01_WelcomeScreen.vue'
import HowToUseScreen from './components/03_HowToUseScreen.vue'
import TermsAgreementScreen from './components/04_TermsAgreementScreen.vue'
import CameraCapture from './components/05_CameraCapture.vue'

const currentScreen = ref('welcome')

const progressPercent = computed(() => {
  switch (currentScreen.value) {
    case 'welcome':
      return 0
    case 'how-to-use':
      return 15
    case 'terms-agreement':
      return 30
    case 'camera-capture':
      return 75
    default:
      return 0
  }
})

const handlePhotoCaptured = (base64Image) => {
  console.log('사진 촬영 완료')
}
</script>

<template>
  <!-- Progress Bar (separated from modal) -->
  <div class="progress-bar-wrapper">
    <div class="progress-bar">
      <div class="progress-fill" :style="{ width: progressPercent + '%' }"></div>
    </div>
  </div>
  
  <div class="app-modal">
    <!-- 현재 화면 값에 따른 동적 화면들 -->
    <WelcomeScreen v-if="currentScreen === 'welcome'" @start="currentScreen = 'how-to-use'" />
    <HowToUseScreen v-if="currentScreen === 'how-to-use'" @next="currentScreen = 'terms-agreement'" />
    <TermsAgreementScreen v-if="currentScreen === 'terms-agreement'" @next="currentScreen = 'camera-capture'" />
    <CameraCapture v-if="currentScreen === 'camera-capture'" @image-captured="handlePhotoCaptured" />
  </div>
</template>

<style>
/* 완전한 CSS 리셋 */
*,
*::before,
*::after {
  box-sizing: border-box;
  margin: 0;
  padding: 0;
}

html,
body {
  margin: 0 !important;
  padding: 0 !important;
  height: 100vh !important;
  width: 100vw !important;
  overflow: hidden !important;
  background: white !important;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
}

/* 브라우저 개발 도구 UI 요소 숨기기 */
[data-v-app] > div:not(.app-modal):not(.progress-bar-wrapper),
.vue-devtools,
#__vconsole,
.vconsole-panel,
.vconsole-topbar {
  display: none !important;
}

/* 떠있는 UI 요소들 숨기기 */
div[style*="position: fixed"]:not(.progress-bar-wrapper):not(.app-modal),
div[style*="position:fixed"]:not(.progress-bar-wrapper):not(.app-modal) {
  display: none !important;
}

/* main.css 스타일 덮어쓰기 */
#app {
  max-width: none !important;
  margin: 0 !important;
  padding: 0 !important;
  font-weight: normal;
  width: 100vw !important;
  height: 100vh !important;
  height: 100dvh !important; /* Dynamic viewport height for mobile */
  position: relative !important;
  overflow: hidden !important;
  box-sizing: border-box !important;
}

/* 진행률 바 */
.progress-bar-wrapper {
  width: 90%;
  padding: 0;
  margin: 10px auto 0 auto;
  background: transparent;
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  z-index: 1000;
}

.progress-bar {
  width: 100%;
  height: 2px;
  background: #E5E7EB;
  border-radius: 1px;
  overflow: hidden;
}

.progress-fill {
  height: 100%;
  background: #7C3AED;
  border-radius: 2px;
  transition: width 0.3s ease;
}

/* 모달 컨테이너 (앱용) */
.app-modal {
  width: 100vw;
  height: 100vh;
  height: 100dvh; /* Dynamic viewport height for mobile */
  background: white;
  display: flex;
  flex-direction: column;
  align-items: stretch;
  justify-content: flex-start;
  margin: 0;
  overflow: hidden;
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  padding: 0;
  box-sizing: border-box;
}

/* 고객 콘텐츠 스타일 */
.customer-content {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 100%;
  padding: 20px;
  text-align: center;
}

.customer-content h1 {
  font-size: 2rem;
  margin-bottom: 1rem;
  color: #333;
}

.customer-content p {
  font-size: 1.1rem;
  color: #666;
}

/* 모바일 스타일 */
@media (max-width: 480px) {
  .app-modal {
    width: 100vw;
    height: 100vh;
    margin: 0;
    padding: 0;
  }
}
</style>