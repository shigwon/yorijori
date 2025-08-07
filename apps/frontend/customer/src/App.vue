<script setup>
import { computed, ref } from 'vue'
import { useRoute } from 'vue-router'
import FaceRecognitionModal from './components/06_FaceRecognitionModal.vue'

const route = useRoute()

// 모달 상태 관리
const showFaceRecognitionModal = ref(false)
const capturedImage = ref('')
const deliveryLocation = ref(null)
const deliveryAddress = ref('')
const showFoodCompartment = ref(false)
const showSurveyScreen = ref(false)
const showChatbot = ref(false)

// 현재 라우트의 progress 메타데이터를 기반으로 진행률 계산
const progressPercent = computed(() => {
  return route.meta.progress || 0
})

// 모달 표시 함수들
const openFaceRecognitionModal = (imageData) => {
  console.log('얼굴 인식 모달 표시 함수 호출됨')
  console.log('받은 이미지 데이터:', imageData ? '있음' : '없음')
  if (imageData) {
    capturedImage.value = `data:image/jpeg;base64,${imageData}`
  }
  showFaceRecognitionModal.value = true
  console.log('showFaceRecognitionModal.value = true 설정 완료')
}

const closeFaceRecognitionModal = () => {
  showFaceRecognitionModal.value = false
}

const openFoodCompartment = () => {
  showFoodCompartment.value = true
}

const closeFoodCompartment = () => {
  showFoodCompartment.value = false
}

const openSurveyScreen = () => {
  showSurveyScreen.value = true
}

const closeSurveyScreen = () => {
  showSurveyScreen.value = false
}

const openChatbot = () => {
  showChatbot.value = true
}

const closeChatbot = () => {
  showChatbot.value = false
}

// 전역으로 모달 함수들을 제공
window.openFaceRecognitionModal = openFaceRecognitionModal
window.closeFaceRecognitionModal = closeFaceRecognitionModal
window.openFoodCompartment = openFoodCompartment
window.closeFoodCompartment = closeFoodCompartment
window.openSurveyScreen = openSurveyScreen
window.closeSurveyScreen = closeSurveyScreen
window.openChatbot = openChatbot
window.closeChatbot = closeChatbot

// 디버깅용 로그
console.log('App.vue에서 전역 함수 등록 완료')
console.log('window.openFaceRecognitionModal:', typeof window.openFaceRecognitionModal)
console.log('window.openFaceRecognitionModal 함수 내용:', window.openFaceRecognitionModal.toString())
</script>

<template>
  <!-- Progress Bar (separated from modal) -->
  <div class="progress-bar-wrapper" v-if="!showFoodCompartment && !showSurveyScreen && !showChatbot">
    <div class="progress-bar">
      <div class="progress-fill" :style="{ width: progressPercent + '%' }"></div>
    </div>
  </div>
  
  <div class="app-modal">
    <router-view />
  </div>

  <!-- 모달들 -->
  <FaceRecognitionModal 
    v-if="showFaceRecognitionModal" 
    :captured-image="capturedImage"
    @previous="closeFaceRecognitionModal"
    @next="closeFaceRecognitionModal"
  />
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