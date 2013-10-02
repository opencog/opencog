#include "AttentionBank.h"

#include <opencog/util/Config.h>

using namespace opencog;

AttentionBank::AttentionBank()
{
    startingFundsSTI = fundsSTI = config().get_int("STARTING_STI_FUNDS");
    startingFundsLTI = fundsLTI = config().get_int("STARTING_LTI_FUNDS");
    attentionalFocusBoundary = 1;
}

AttentionBank::~AttentionBank()
{
    fundsSTI = config().get_int("STARTING_STI_FUNDS");
}

long AttentionBank::getTotalSTI() const {
    return startingFundsSTI - fundsSTI;
}

long AttentionBank::getTotalLTI() const {
    return startingFundsLTI - fundsLTI;
}

void AttentionBank::setAV(AttentionValueHolderPtr avh, const AttentionValue& av)
{
    boost::mutex::scoped_lock lock(lock_funds);
    const AttentionValue& oldAV = avh->getAttentionValue();
    // Add the old attention values to the AtomSpace funds and
    // subtract the new attention values from the AtomSpace funds
    fundsSTI += (oldAV.getSTI() - av.getSTI());
    fundsLTI += (oldAV.getLTI() - av.getLTI());

    avh->setAttentionValue(av);
}

void AttentionBank::setSTI(AttentionValueHolderPtr avh, AttentionValue::sti_t stiValue)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(stiValue, currentAv.getLTI(), currentAv.getVLTI()));
}

void AttentionBank::setLTI(AttentionValueHolderPtr avh, AttentionValue::lti_t ltiValue)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(currentAv.getSTI(), ltiValue, currentAv.getVLTI()));
}

void AttentionBank::incVLTI(AttentionValueHolderPtr avh)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(currentAv.getSTI(), currentAv.getLTI(), currentAv.getVLTI()+1));
}

void AttentionBank::decVLTI(AttentionValueHolderPtr avh)
{
    const AttentionValue& currentAv = getAV(avh);
    AttentionValue::vlti_t vlti = currentAv.getVLTI();
    //we only want to decrement the vlti if it's not already disposable.
    if(vlti!=AttentionValue::DISPOSABLE) vlti--;
    setAV(avh, AttentionValue(currentAv.getSTI(), currentAv.getLTI(), vlti));
}

AttentionValue::sti_t AttentionBank::getSTI(AttentionValueHolderPtr avh) const
{
    return avh->getAttentionValue().getSTI();
}

AttentionValue::lti_t AttentionBank::getLTI(AttentionValueHolderPtr avh) const
{
    return avh->getAttentionValue().getLTI();
}

AttentionValue::vlti_t AttentionBank::getVLTI(AttentionValueHolderPtr avh) const
{
    return avh->getAttentionValue().getVLTI();
}

long AttentionBank::getSTIFunds() const
{
    boost::mutex::scoped_lock lock(lock_funds);
    return fundsSTI;
}

long AttentionBank::getLTIFunds() const
{
    boost::mutex::scoped_lock lock(lock_funds);
    return fundsLTI;
}

long AttentionBank::updateSTIFunds(AttentionValue::sti_t diff)
{
    boost::mutex::scoped_lock lock(lock_funds);
    fundsSTI+=diff;
    return fundsSTI;
}

long AttentionBank::updateLTIFunds(AttentionValue::lti_t diff)
{
    boost::mutex::scoped_lock lock(lock_funds);
    fundsLTI+=diff;
    return fundsLTI;
}

void AttentionBank::updateMaxSTI(AttentionValue::sti_t m)
{
    boost::mutex::scoped_lock lock(lock_maxSTI);
    maxSTI.update(m);
}

void AttentionBank::updateMinSTI(AttentionValue::sti_t m)
{
    boost::mutex::scoped_lock lock(lock_minSTI);
    minSTI.update(m);
}

AttentionValue::sti_t AttentionBank::getMaxSTI(bool average) const
{
    boost::mutex::scoped_lock lock(lock_maxSTI);
    if (average) {
        return (AttentionValue::sti_t) maxSTI.recent;
    } else {
        return maxSTI.val;
    }
}

AttentionValue::sti_t AttentionBank::getMinSTI(bool average) const
{
    boost::mutex::scoped_lock lock(lock_minSTI);
    if (average) {
        return (AttentionValue::sti_t) minSTI.recent;
    } else {
        return minSTI.val;
    }
}

AttentionValue::sti_t AttentionBank::getAttentionalFocusBoundary() const
{
    return attentionalFocusBoundary;
}

AttentionValue::sti_t AttentionBank::setAttentionalFocusBoundary(AttentionValue::sti_t boundary)
{
    attentionalFocusBoundary = boundary;
    return boundary;
}

const AttentionValue& AttentionBank::getAV(AttentionValueHolderPtr avh) const
{
    return avh->getAttentionValue();
}
