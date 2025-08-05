import { ref } from 'vue'

const progressPercent = ref(0)

function setProgressPercent(percent) {
  progressPercent.value = percent
}

const currentScreen = ref('how-to-use')

const goToScanOption = () => {
  currentScreen.value = 'scan-option'
  console.log('화면 전환:', currentScreen.value)
}

export const useAppState = () => {
  const goToHowToUse = () => {
    currentScreen.value = 'how-to-use'
    console.log('화면 전환:', currentScreen.value)
  }

  return {
    currentScreen,
    goToHowToUse,
    goToScanOption,
    progressPercent,
    setProgressPercent,
  }
}