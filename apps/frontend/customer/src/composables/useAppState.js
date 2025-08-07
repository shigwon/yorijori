import { ref } from 'vue'
import { useRouter } from 'vue-router'

const capturedImage = ref('')
const deliveryLocation = ref(null)
const deliveryAddress = ref('')

export const useAppState = () => {
  const router = useRouter()

  const goToWelcome = () => {
    router.push('/welcome')
    console.log('화면 전환: welcome')
  }

  const goToHowToUse = () => {
    router.push('/how-to-use')
    console.log('화면 전환: how-to-use')
  }

  const goToTermsAgreement = () => {
    router.push('/terms-agreement')
    console.log('화면 전환: terms-agreement')
  }

  const goToPhotoSelection = () => {
    router.push('/photo-selection')
    console.log('화면 전환: photo-selection')
  }

  const goToCameraCapture = () => {
    router.push('/camera-capture')
    console.log('화면 전환: camera-capture')
  }

  const goToLocationSetting = () => {
    router.push('/location-setting')
    console.log('화면 전환: location-setting')
  }

  const goToDeliveryTracking = () => {
    router.push('/delivery-tracking')
    console.log('화면 전환: delivery-tracking')
  }

  const goToFoodCompartment = () => {
    router.push('/food-compartment')
    console.log('화면 전환: food-compartment')
  }

  const goToSurvey = () => {
    router.push('/survey')
    console.log('화면 전환: survey')
  }

  const openFaceRecognitionModal = (imageData) => {
    window.openFaceRecognitionModal(imageData)
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
    closeChatbot,
  }
}
