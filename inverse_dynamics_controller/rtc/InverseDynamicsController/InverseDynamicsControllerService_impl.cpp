#include "InverseDynamicsControllerService_impl.h"
#include "InverseDynamicsController.h"

InverseDynamicsControllerService_impl::InverseDynamicsControllerService_impl()
{
}

InverseDynamicsControllerService_impl::~InverseDynamicsControllerService_impl()
{
}

CORBA::Boolean InverseDynamicsControllerService_impl::startInverseDynamicsController()
{
  return this->comp_->startInverseDynamicsController();
};

CORBA::Boolean InverseDynamicsControllerService_impl::stopInverseDynamicsController()
{
  return this->comp_->stopInverseDynamicsController();
};

CORBA::Boolean InverseDynamicsControllerService_impl::setInverseDynamicsControllerParam(const OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam& i_param)
{
  return this->comp_->setInverseDynamicsControllerParam(i_param);
};

CORBA::Boolean InverseDynamicsControllerService_impl::getInverseDynamicsControllerParam(OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam_out i_param)
{
  i_param = new OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam();
  return this->comp_->getInverseDynamicsControllerParam(*i_param);
};

void InverseDynamicsControllerService_impl::setComp(InverseDynamicsController *i_comp)
{
  this->comp_ = i_comp;
}
