<template>
  <v-dialog v-model="isOpen" max-width="600px" persistent>
    <v-card class="settings-modal" dark>
      <v-card-title class="settings-header">
        <v-icon class="settings-icon">mdi-cog</v-icon>
        <span class="settings-title">설정</span>
        <v-spacer></v-spacer>
        <v-btn icon @click="closeModal" class="close-btn">
          <v-icon>mdi-close</v-icon>
        </v-btn>
      </v-card-title>

      <v-card-text class="settings-content">
        <v-tabs v-model="activeTab" color="#3dade5" class="settings-tabs">
          <v-tab value="general">
            <v-icon left>mdi-cog-outline</v-icon>
            일반
          </v-tab>
          <v-tab value="notifications">
            <v-icon left>mdi-bell-outline</v-icon>
            알림
          </v-tab>
          <v-tab value="appearance">
            <v-icon left>mdi-palette-outline</v-icon>
            외관
          </v-tab>
          <v-tab value="security">
            <v-icon left>mdi-shield-outline</v-icon>
            보안
          </v-tab>
        </v-tabs>

        <v-window v-model="activeTab" class="settings-window">
          <!-- 일반 설정 -->
          <v-window-item value="general">
            <div class="tab-content">
              <h3 class="section-title">일반 설정</h3>
              
              <v-list class="settings-list">
                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>언어</v-list-item-title>
                    <v-list-item-subtitle>인터페이스 언어를 선택하세요</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-select
                      v-model="settings.language"
                      :items="['한국어', 'English', '日本語']"
                      variant="outlined"
                      density="compact"
                      hide-details
                    ></v-select>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>시간대</v-list-item-title>
                    <v-list-item-subtitle>표시할 시간대를 선택하세요</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-select
                      v-model="settings.timezone"
                      :items="['Asia/Seoul', 'UTC', 'America/New_York']"
                      variant="outlined"
                      density="compact"
                      hide-details
                    ></v-select>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>자동 새로고침</v-list-item-title>
                    <v-list-item-subtitle>데이터 자동 새로고침 간격</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.autoRefresh"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>
              </v-list>
            </div>
          </v-window-item>

          <!-- 알림 설정 -->
          <v-window-item value="notifications">
            <div class="tab-content">
              <h3 class="section-title">알림 설정</h3>
              
              <v-list class="settings-list">
                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>이메일 알림</v-list-item-title>
                    <v-list-item-subtitle>중요한 이벤트에 대한 이메일 알림</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.emailNotifications"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>로봇 상태 알림</v-list-item-title>
                    <v-list-item-subtitle>로봇 상태 변경 시 알림</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.robotStatusNotifications"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>시스템 로그 알림</v-list-item-title>
                    <v-list-item-subtitle>중요한 시스템 로그 발생 시 알림</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.systemLogNotifications"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>알림음</v-list-item-title>
                    <v-list-item-subtitle>알림 발생 시 소리 재생</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.soundNotifications"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>
              </v-list>
            </div>
          </v-window-item>

          <!-- 외관 설정 -->
          <v-window-item value="appearance">
            <div class="tab-content">
              <h3 class="section-title">외관 설정</h3>
              
              <v-list class="settings-list">
                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>테마</v-list-item-title>
                    <v-list-item-subtitle>인터페이스 테마를 선택하세요</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-select
                      v-model="settings.theme"
                      :items="['다크', '라이트', '시스템']"
                      variant="outlined"
                      density="compact"
                      hide-details
                    ></v-select>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>애니메이션</v-list-item-title>
                    <v-list-item-subtitle>인터페이스 애니메이션 활성화</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.animations"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>컴팩트 모드</v-list-item-title>
                    <v-list-item-subtitle>더 조밀한 레이아웃 사용</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.compactMode"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>
              </v-list>
            </div>
          </v-window-item>

          <!-- 보안 설정 -->
          <v-window-item value="security">
            <div class="tab-content">
              <h3 class="section-title">보안 설정</h3>
              
              <v-list class="settings-list">
                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>2단계 인증</v-list-item-title>
                    <v-list-item-subtitle>추가 보안을 위한 2단계 인증</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.twoFactorAuth"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>세션 타임아웃</v-list-item-title>
                    <v-list-item-subtitle>자동 로그아웃 시간 설정</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-select
                      v-model="settings.sessionTimeout"
                      :items="['15분', '30분', '1시간', '4시간', '무제한']"
                      variant="outlined"
                      density="compact"
                      hide-details
                    ></v-select>
                  </v-list-item-action>
                </v-list-item>

                <v-list-item>
                  <v-list-item-content>
                    <v-list-item-title>로그인 알림</v-list-item-title>
                    <v-list-item-subtitle>새로운 로그인 시 알림</v-list-item-subtitle>
                  </v-list-item-content>
                  <v-list-item-action>
                    <v-switch
                      v-model="settings.loginNotifications"
                      color="#3dade5"
                      hide-details
                    ></v-switch>
                  </v-list-item-action>
                </v-list-item>
              </v-list>
            </div>
          </v-window-item>
        </v-window>
      </v-card-text>

      <v-card-actions class="settings-actions">
        <v-spacer></v-spacer>
        <v-btn
          color="grey"
          variant="text"
          @click="resetSettings"
          class="reset-btn"
        >
          초기화
        </v-btn>
        <v-btn
          color="grey"
          variant="text"
          @click="closeModal"
          class="cancel-btn"
        >
          취소
        </v-btn>
        <v-btn
          color="#3dade5"
          @click="saveSettings"
          class="save-btn"
        >
          저장
        </v-btn>
      </v-card-actions>
    </v-card>
  </v-dialog>
</template>

<script setup>
import { ref, reactive, watch } from 'vue'

const props = defineProps({
  modelValue: {
    type: Boolean,
    default: false
  }
})

const emit = defineEmits(['update:modelValue', 'settings-saved'])

const isOpen = ref(false)
const activeTab = ref('general')

// 설정 데이터
const settings = reactive({
  language: '한국어',
  timezone: 'Asia/Seoul',
  autoRefresh: true,
  emailNotifications: true,
  robotStatusNotifications: true,
  systemLogNotifications: false,
  soundNotifications: true,
  theme: '다크',
  animations: true,
  compactMode: false,
  twoFactorAuth: false,
  sessionTimeout: '30분',
  loginNotifications: true
})

// 모달 열기/닫기
watch(() => props.modelValue, (newVal) => {
  isOpen.value = newVal
})

watch(isOpen, (newVal) => {
  emit('update:modelValue', newVal)
})

const closeModal = () => {
  isOpen.value = false
}

const saveSettings = () => {
  // 설정 저장 로직
  console.log('설정 저장:', settings)
  emit('settings-saved', settings)
  closeModal()
}

const resetSettings = () => {
  // 기본값으로 초기화
  Object.assign(settings, {
    language: '한국어',
    timezone: 'Asia/Seoul',
    autoRefresh: true,
    emailNotifications: true,
    robotStatusNotifications: true,
    systemLogNotifications: false,
    soundNotifications: true,
    theme: '다크',
    animations: true,
    compactMode: false,
    twoFactorAuth: false,
    sessionTimeout: '30분',
    loginNotifications: true
  })
}
</script>

<style scoped>
.settings-modal {
  background-color: #1a1f2e !important;
  color: #ffffff !important;
}

.settings-header {
  background-color: #2a2f3e;
  border-bottom: 1px solid #3a3f4e;
  padding: 20px 24px;
}

.settings-icon {
  color: #3dade5;
  margin-right: 12px;
  font-size: 24px;
}

.settings-title {
  font-size: 20px;
  font-weight: 600;
  color: #ffffff;
}

.close-btn {
  color: #8a92a6;
}

.close-btn:hover {
  color: #ffffff;
}

.settings-content {
  padding: 0;
}

.settings-tabs {
  background-color: #2a2f3e;
  border-bottom: 1px solid #3a3f4e;
}

.settings-window {
  min-height: 400px;
}

.tab-content {
  padding: 24px;
}

.section-title {
  font-size: 18px;
  font-weight: 600;
  color: #ffffff;
  margin-bottom: 20px;
  padding-bottom: 8px;
  border-bottom: 2px solid #3dade5;
}

.settings-list {
  background-color: transparent;
}

.settings-list .v-list-item {
  border-bottom: 1px solid #3a3f4e;
  padding: 16px 0;
}

.settings-list .v-list-item:last-child {
  border-bottom: none;
}

.settings-list .v-list-item-title {
  color: #ffffff;
  font-weight: 500;
}

.settings-list .v-list-item-subtitle {
  color: #8a92a6;
  font-size: 12px;
}

.settings-actions {
  background-color: #2a2f3e;
  border-top: 1px solid #3a3f4e;
  padding: 16px 24px;
}

.reset-btn {
  color: #ff6b6b;
}

.cancel-btn {
  color: #8a92a6;
}

.save-btn {
  background-color: #3dade5;
  color: #ffffff;
  font-weight: 500;
}

.save-btn:hover {
  background-color: #2b8bc7;
}

/* Vuetify 컴포넌트 스타일 오버라이드 */
:deep(.v-tab) {
  color: #8a92a6;
  font-weight: 500;
}

:deep(.v-tab--selected) {
  color: #3dade5;
}

:deep(.v-tabs__bar) {
  background-color: #2a2f3e;
}

:deep(.v-tabs__slider) {
  background-color: #3dade5;
}

:deep(.v-select .v-field) {
  background-color: #2a2f3e;
  border-color: #3a3f4e;
}

:deep(.v-select .v-field__input) {
  color: #ffffff;
}

:deep(.v-switch .v-selection-control) {
  color: #3dade5;
}

:deep(.v-list) {
  background-color: transparent;
}

:deep(.v-list-item) {
  background-color: transparent;
}

:deep(.v-list-item:hover) {
  background-color: #2a2f3e;
}
</style>
