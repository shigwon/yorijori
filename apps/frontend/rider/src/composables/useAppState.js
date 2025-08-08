import { ref } from 'vue'
import { useRouter } from 'vue-router'

const progressPercent = ref(0)
const receiptData = ref({ id: '', tel: '' })
const orderData = ref({ orderNumber: '', safeNumber: '' })

export const useAppState = () => {
  const router = useRouter()

  function setProgressPercent(percent) {
    progressPercent.value = percent
  }

  const goToHowToUse = () => {
    router.push('/rider/how-to-use')
    console.log('화면 전환: how-to-use')
  }

  const goToScanOption = () => {
    router.push('/rider/scan-option')
    console.log('화면 전환: scan-option')
  }

  const goToReceiptScan = () => {
    router.push('/rider/receipt-scan')
    console.log('화면 전환: receipt-scan')
  }

  const goToManualInput = () => {
    router.push('/rider/manual-input')
    console.log('화면 전환: manual-input')
  }

  const goToManualConfirm = (orderNumber, safeNumber) => {
    orderData.value = { orderNumber, safeNumber }
    router.push('/rider/manual-confirm')
    console.log('화면 전환: manual-confirm')
  }

  const goToLocationRequest = () => {
    router.push('/rider/location-request')
    console.log('화면 전환: location-request')
  }

  const goToPhotoCapture = () => {
    router.push('/rider/photo-capture')
    console.log('화면 전환: photo-capture')
  }

  const goToComplete = () => {
    router.push('/rider/complete')
    console.log('화면 전환: complete')
  }

  const goToOnboarding = () => {
    router.push('/rider/how-to-use')
    console.log('화면 전환: onboarding -> how-to-use')
  }

  const goToLoadingModal = () => {
    window.openLoadingModal()
    console.log('로딩 모달 표시')
  }

  const goToScanConfirmModal = () => {
    window.openScanConfirmModal()
    console.log('스캔 확인 모달 표시')
  }

  return {
    receiptData,
    orderData,
    goToHowToUse,
    goToScanOption,
    goToReceiptScan,
    goToManualInput,
    goToManualConfirm,
    goToLocationRequest,
    goToPhotoCapture,
    goToComplete,
    goToOnboarding,
    goToLoadingModal,
    goToScanConfirmModal,
    progressPercent,
    setProgressPercent,
  }
}
