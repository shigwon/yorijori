import { ref } from 'vue'
import { useRouter } from 'vue-router'

// 전역 상태 변수들
const capturedImage = ref('')
const deliveryLocation = ref(null)
const deliveryAddress = ref('')
const orderCode = ref('') // 주문번호 저장

// 새로운 상태 변수들 추가
const robotId = ref('')      // 로봇 ID
const orderId = ref('')      // 주문 ID
const sectionNum = ref('')   // 음식함 번호

// URL에서 모든 파라미터 파싱하는 함수 (전역 함수)
export const parseUrlParameters = () => {
  const urlParams = new URLSearchParams(window.location.search)
  
  // 각 파라미터 추출
  const code = urlParams.get('code')
  const robotIdParam = urlParams.get('robotId')
  const orderIdParam = urlParams.get('orderId')
  const sectionNumParam = urlParams.get('sectionNum')
  
  // 상태에 저장
  if (code) {
    orderCode.value = code
    console.log('주문번호 저장:', code)
  }
  
  if (robotIdParam) {
    robotId.value = robotIdParam
    console.log('로봇 ID 저장:', robotIdParam)
  }
  
  if (orderIdParam) {
    orderId.value = orderIdParam
    console.log('주문 ID 저장:', orderIdParam)
  }
  
  if (sectionNumParam) {
    sectionNum.value = sectionNumParam
    console.log('음식함 번호 저장:', sectionNumParam)
  }
  
  // 파싱된 정보 반환
  return {
    code: orderCode.value,
    robotId: robotId.value,
    orderId: orderId.value,
    sectionNum: sectionNum.value
  }
}

export const useAppState = () => {
  const router = useRouter()

  const goToWelcome = () => {
    router.push('/customer/welcome')
    console.log('화면 전환: welcome')
  }

  const goToHowToUse = () => {
    router.push('/customer/how-to-use')
    console.log('화면 전환: how-to-use')
  }

  const goToTermsAgreement = () => {
    router.push('/customer/terms-agreement')
    console.log('화면 전환: terms-agreement')
  }

  const goToPhotoSelection = () => {
    router.push('/customer/photo-selection')
    console.log('화면 전환: photo-selection')
  }

  const goToCameraCapture = () => {
    router.push('/customer/camera-capture')
    console.log('화면 전환: camera-capture')
  }

  const goToLocationSetting = () => {
    router.push('/customer/location-setting')
    console.log('화면 전환: location-setting')
  }

  const goToDeliveryTracking = () => {
    router.push('/customer/delivery-tracking')
    console.log('화면 전환: delivery-tracking')
  }

  const goToFoodCompartment = () => {
    router.push('/customer/food-compartment')
    console.log('화면 전환: food-compartment')
  }

  const goToSurvey = () => {
    router.push('/customer/survey')
    console.log('화면 전환: survey')
  }

  const openFaceRecognitionModal = (imageData) => {
    if (window.openFaceRecognitionModal) {
      window.openFaceRecognitionModal(imageData)
    }
    console.log('얼굴 인식 모달 표시')
  }

  const closeFaceRecognitionModal = () => {
    window.closeFaceRecognitionModal()
    console.log('얼굴 인식 모달 닫기')
  }

  const openFoodCompartment = () => {
    window.openFoodCompartment()
    console.log('음식함 화면 표시')
  }

  const closeFoodCompartment = () => {
    window.closeFoodCompartment()
    console.log('음식함 화면 닫기')
  }

  const openSurveyScreen = () => {
    window.openSurveyScreen()
    console.log('설문조사 화면 표시')
  }

  const closeSurveyScreen = () => {
    window.closeSurveyScreen()
    console.log('설문조사 화면 닫기')
  }

  const openChatbot = () => {
    window.openChatbot()
    console.log('챗봇 표시')
  }

  const closeChatbot = () => {
    window.closeChatbot()
    console.log('챗봇 닫기')
  }

  return {
    capturedImage,
    deliveryLocation,
    deliveryAddress,
    orderCode,
    robotId,
    orderId,
    sectionNum,
    goToWelcome,
    goToHowToUse,
    goToTermsAgreement,
    goToPhotoSelection,
    goToCameraCapture,
    goToLocationSetting,
    goToDeliveryTracking,
    goToFoodCompartment,
    goToSurvey,
    openFaceRecognitionModal,
    closeFaceRecognitionModal,
    openFoodCompartment,
    closeFoodCompartment,
    openSurveyScreen,
    closeSurveyScreen,
    openChatbot,
    closeChatbot
  }
}
