<script setup>
// 고객 앱 상태 관리
import { ref, computed } from 'vue'
import WelcomeScreen from './components/01_WelcomeScreen.vue'
import HowToUseScreen from './components/02_HowToUseScreen.vue'
import TermsAgreementScreen from './components/03_TermsAgreementScreen.vue'
import PhotoSelectionScreen from './components/04_PhotoSelectionScreen.vue'
import CameraCapture from './components/05_CameraCapture.vue'
import FaceRecognitionModal from './components/06_FaceRecognitionModal.vue'
import LocationSettingScreen from './components/07_LocationSettingScreen.vue'
import DeliveryTrackingScreen from './components/08_DeliveryTrackingScreen.vue'
import DeliveryCompleteModal from './components/09_DeliveryCompleteModal.vue'
import FoodCompartmentScreen from './components/10_FoodCompartmentScreen.vue'
import SurveyScreen from './components/11_SurveyScreen.vue'

const currentScreen = ref('welcome')
const showFaceRecognitionModal = ref(false)
const capturedImage = ref('')
const deliveryLocation = ref(null)
const deliveryAddress = ref('')
const showFoodCompartment = ref(false)
const showSurveyScreen = ref(false)

const progressPercent = computed(() => {
  switch (currentScreen.value) {
    case 'welcome':
      return 0
    case 'how-to-use':
      return 15
    case 'terms-agreement':
      return 30
    case 'photo-selection':
      return 45
    case 'camera-capture':
      return 60
    case 'location-setting':
      return 75
    case 'delivery-tracking':
      return 90
    default:
      return 0
  }
})

const handlePhotoCaptured = (base64Image) => {
  console.log('사진 촬영 완료')
}

const handleShowFaceRecognition = (imageData) => {
  console.log('얼굴 인식 모달 표시 함수 호출됨')
  if (imageData) {
    capturedImage.value = `data:image/jpeg;base64,${imageData}`
  }
  setTimeout(() => {
    showFaceRecognitionModal.value = true
  }, 50)
}

const handleLocationConfirmed = (locationData) => {
  console.log('위치 설정 완료:', locationData)
  deliveryLocation.value = locationData.location
  deliveryAddress.value = locationData.address
  currentScreen.value = 'delivery-tracking'
}

const handleCompartmentConfirm = () => {
  console.log('확인 버튼 클릭')
  showFoodCompartment.value = false
  // 페이지 나가기
  window.close()
}

const handleSurvey = () => {
  console.log('고객만족도 평가 클릭')
  showFoodCompartment.value = false
  showSurveyScreen.value = true
}
</script>

<template>
  <!-- Progress Bar (separated from modal) -->
  <div class="progress-bar-wrapper" v-if="!showFoodCompartment && !showSurveyScreen">
    <div class="progress-bar">
      <div class="progress-fill" :style="{ width: progressPercent + '%' }"></div>
    </div>
  </div>
  
  <div class="app-modal">
    <!-- 현재 화면 값에 따른 동적 화면들 -->
    <WelcomeScreen v-if="currentScreen === 'welcome'" @start="currentScreen = 'how-to-use'" />
    <HowToUseScreen v-if="currentScreen === 'how-to-use'" @next="currentScreen = 'terms-agreement'" />
    <TermsAgreementScreen v-if="currentScreen === 'terms-agreement'" @next="currentScreen = 'photo-selection'" />
    <PhotoSelectionScreen v-if="currentScreen === 'photo-selection'" @take-selfie="currentScreen = 'camera-capture'" @show-face-recognition="handleShowFaceRecognition" />
    <CameraCapture v-if="currentScreen === 'camera-capture'" @image-captured="handlePhotoCaptured" @show-face-recognition="handleShowFaceRecognition" />
    
    <!-- 얼굴 인식 모달 -->
    <FaceRecognitionModal 
      v-if="showFaceRecognitionModal" 
      :captured-image="capturedImage"
      @previous="showFaceRecognitionModal = false"
      @next="currentScreen = 'location-setting'; showFaceRecognitionModal = false"
    />
    
    <LocationSettingScreen 
      v-if="currentScreen === 'location-setting'" 
      :face-image="capturedImage"
      @location-confirmed="handleLocationConfirmed" 
    />
    
    <DeliveryTrackingScreen 
      v-if="currentScreen === 'delivery-tracking'" 
      :delivery-location="deliveryLocation"
      :delivery-address="deliveryAddress"
      :face-image="capturedImage"
      @delivery-completed="currentScreen = 'photo-selection'"
      @show-compartment="showFoodCompartment = true"
    />
    
    <!-- 음식함 화면 -->
    <FoodCompartmentScreen 
      v-if="showFoodCompartment" 
      @confirm="handleCompartmentConfirm"
      @survey="handleSurvey"
    />
    
    <!-- 설문조사 화면 -->
    <SurveyScreen 
      v-if="showSurveyScreen" 
    />
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