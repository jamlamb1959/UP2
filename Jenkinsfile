pipeline {
  agent {
    node {
      label 'esp32'
    }

  }
  stages {
    stage('build') {
      steps {
        sh '''
export PATH=${PATH}:${HOME}/.local/bin

echo "BRANCH_NAME: ${BRANCH_NAME}"

make
'''
      }
    }

  }
}